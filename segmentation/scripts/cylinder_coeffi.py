#!/usr/bin/env python


import rospy
import numpy as np

import yaml
import sys

from segmentation.msg import processed_cylinder_data
from segmentation.msg import cylinder_data



# The calibration result might have some constant offset
x_offset = 0.0
y_offset = -0.001
z_offset = 0.001



class cylinder_coefficients:
    def __init__(self):


        self.cylinder_data_pub = rospy.Publisher('processed_cylinder_data', processed_cylinder_data,queue_size=10)
        self.cylinder_data_sub = rospy.Subscriber('cylinder_data',cylinder_data,self.cylinder_callback)


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


    def cylinder_callback(self,data):
        data = np.array(data.cylinder_data).astype(float)
        cylinder_coefficients = data[:7]
        center_in_rgb = data[7:10]
        center_in_world = self.point_in_rgb_to_world(center_in_rgb.reshape(3,1))
        central_axis_direction = self.get_axis_in_world(cylinder_coefficients)
        radius = cylinder_coefficients[6]

        msg = processed_cylinder_data()
        msg.center = center_in_world
        msg.central_axis = central_axis_direction
        msg.radius = radius
        self.cylinder_data_pub.publish(msg)


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

    def get_axis_in_world(self, cylinder_coefficients):
        # cylinder_coefficients are the coefficients from pcl SACSegmentationFromNormals
        point_in_axis = np.zeros(3)
        axis_one_end = np.zeros(3)
        axis_another_end = np.zeros(3)
        point_in_axis = cylinder_coefficients[:3]
        axis_another_end = cylinder_coefficients[3:6]

        point_in_axis_in_world = self.point_in_rgb_to_world(point_in_axis.reshape(3,1))
        axis_one_end_in_world = self.point_in_rgb_to_world(axis_one_end.reshape(3,1))
        axis_another_end_in_world = self.point_in_rgb_to_world(axis_another_end.reshape(3,1))

        central_axis_direction = axis_another_end_in_world - axis_one_end_in_world
        point_in_central_axis = point_in_axis_in_world

        central_axis_direction = central_axis_direction / np.linalg.norm(central_axis_direction)



        return central_axis_direction


if __name__ == "__main__":
    rospy.init_node('cylinder_coefficients_transformation')
    ic = cylinder_coefficients()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

