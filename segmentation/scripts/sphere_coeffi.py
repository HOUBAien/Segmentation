#!/usr/bin/env python


import rospy
import numpy as np

import yaml
import sys

from segmentation.msg import processed_sphere_data
from segmentation.msg import sphere_data



# The calibration result might have some constant offset
x_offset = 0.0
y_offset = -0.001
z_offset = 0.001



class sphere_coefficients:
    def __init__(self):


        self.sphere_data_pub = rospy.Publisher('processed_sphere_data', processed_sphere_data,queue_size=10)
        self.sphere_data_sub = rospy.Subscriber('sphere_data',sphere_data,self.sphere_callback)


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


    def sphere_callback(self,data):
        data = np.array(data.sphere_data).astype(float)

        center_in_rgb = data[:3]
        center_in_world = self.point_in_rgb_to_world(center_in_rgb.reshape(3,1))
        radius = data[3]

        msg = processed_sphere_data()
        msg.center = center_in_world
        msg.radius = radius
        self.sphere_data_pub.publish(msg)


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
    rospy.init_node('sphere_coefficients_transformation')
    ic = sphere_coefficients()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

