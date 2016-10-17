#!/usr/bin/env python


import rospy
import numpy as np

import yaml
import sys

from segmentation.msg import processed_box_3D_data
from segmentation.msg import box_3D_data



# The calibration result might have some constant offset
x_offset = 0.0
y_offset = -0.001
z_offset = 0.001


class box_3D_coefficients:
    def __init__(self):


        self.box_3D_data_pub = rospy.Publisher('processed_box_3D_data', processed_box_3D_data,queue_size=10)
        self.box_3D_data_sub = rospy.Subscriber('box_3D_data',box_3D_data,self.box_3D_callback)


        self.rgb_rmat = None
        self.rgb_tvec = None
        self.ir_rmat = None
        self.ir_tvec = None

        self.ir_to_rgb_rmat = None
        self.ir_to_rgb_tvec = None

        self.ir_to_world_tvec = None
        self.ir_to_world_rmat = None
        self.rgb_to_world_rmat = None
        self.rgb_to_world_tvec = None

        self.load_extrinsics()


    def box_3D_callback(self,data):
        center = np.array(data.center).astype(float)
        longestEdgeLen = data.longestEdgeLen
        medianEdgeLen = data.medianEdgeLen
        shortestEdgeLen = data.shortestEdgeLen

        longestEdgeDir = np.array(data.longestEdgeDir).astype(float)
        medianEdgeDir = np.array(data.medianEdgeDir).astype(float)
        shortestEdgeDir = np.array(data.shortestEdgeDir).astype(float)


        center_in_world = self.point_in_rgb_to_world(center.reshape(3,1))

        msg = processed_box_3D_data()
        msg.center = center_in_world
        msg.longestEdgeLen = longestEdgeLen
        msg.medianEdgeLen = medianEdgeLen
        msg.shortestEdgeLen = shortestEdgeLen
        msg.longestEdgeDir = self.get_vector_in_world(longestEdgeDir)
        msg.medianEdgeDir = self.get_vector_in_world(medianEdgeDir)
        msg.shortestEdgeDir = self.get_vector_in_world(shortestEdgeDir)

        self.box_3D_data_pub.publish(msg)

    def get_vector_in_world(self, dir_vector):
        dir_end_in_world = self.point_in_rgb_to_world(dir_vector.reshape(3,1))
        rgb_origin_in_world = self.point_in_rgb_to_world(np.zeros((3,1)))
        dir_vector_in_world = dir_end_in_world - rgb_origin_in_world
        dir_vector_in_world /= np.linalg.norm(dir_vector_in_world)
        return dir_vector_in_world




    def get_angle_to_x_axis(self, long_edge_vector):
        long_edge_in_world = self.point_in_rgb_to_world(long_edge_vector.reshape(3,1))
        rgb_origin_in_world = self.point_in_rgb_to_world(np.zeros((3,1)))
        long_edge_in_world = (long_edge_in_world - rgb_origin_in_world)
        long_edge_in_world /= np.linalg.norm(long_edge_in_world)

        alpha = np.arctan2(long_edge_in_world[1,0], long_edge_in_world[0,0])
        alpha = np.rad2deg(alpha)
        alpha = (alpha + 180) % 180
        return alpha

    def load_extrinsics(self):
       ir_stream = open("/home/chentao/kinect_calibration/ir_camera_pose.yaml", "r")
       ir_doc = yaml.load(ir_stream)
       self.ir_rmat = np.array(ir_doc['rmat']).reshape(3,3)
       self.ir_tvec = np.array(ir_doc['tvec'])
       ir_stream.close()

       rgb_stream = open("/home/chentao/kinect_calibration/rgb_camera_pose.yaml", "r")
       rgb_doc = yaml.load(rgb_stream)
       self.rgb_rmat = np.array(rgb_doc['rmat']).reshape(3,3)
       self.rgb_tvec = np.array(rgb_doc['tvec'])
       rgb_stream.close()

       self.ir_to_world_rmat = self.ir_rmat.T
       self.ir_to_world_tvec = -np.dot(self.ir_rmat.T, self.ir_tvec)

       self.rgb_to_world_rmat = self.rgb_rmat.T
       self.rgb_to_world_tvec = -np.dot(self.rgb_rmat.T, self.rgb_tvec)


    def img_to_world(self, pix_point):
        if self.depth_image == None or self.rgb_image == None:
            return

        # pix_point is (u,v) : the coordinates on the image
        depth_pix_point = np.array([pix_point[0], pix_point[1], 1]) * self.depth_image[pix_point[1], pix_point[0]]
        depth_coord_point = np.dot(np.linalg.inv(self.rgb_mtx), depth_pix_point.reshape(-1,1))

        point_in_world = np.dot(self.rgb_to_world_rmat, depth_coord_point.reshape(-1,1)) + self.rgb_to_world_tvec
        point_in_world[0] += x_offset
        point_in_world[1] += y_offset
        point_in_world[2] += z_offset
        return point_in_world


    def point_in_rgb_to_world(self, point_in_rgb_3d):
        point_in_world = np.dot(self.rgb_to_world_rmat, point_in_rgb_3d) + self.rgb_to_world_tvec

        return point_in_world




if __name__ == "__main__":
    rospy.init_node('box_3D_coefficients_transformation')
    ic = box_3D_coefficients()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

