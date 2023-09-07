#!/usr/bin/env python3
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python import get_resource
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray

import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


class kinematics_processer(Node):
    def __init__(self):
        super().__init__('kinematic_node')


        self.frontcamera_xyz_publisher = self.create_publisher(
            Float32MultiArray,
            'front_camera_xyz',
            10
        )

        self.front_camera_position_sender_timer = self.create_timer(0.5, self.forward_xyz_sender_timer_callback)

        #Definiton of the links, joint and structure
        self.pin_to_front_camera_chain = ikpy.chain.Chain.from_urdf_file("/home/gregorio/ros2_ws/src/forw_inver_kin_urdf_based/models/pin_to_front_camera.urdf")
        print(self.pin_to_front_camera_chain.links)
        print(self.pin_to_front_camera_chain.name)
        print(self.pin_to_front_camera_chain.active_links_mask)
        print(self.pin_to_front_camera_chain._urdf_metadata)               

        self.received_geom_joint_values = [0.0, 0.0, 0.0]

    def forward_xyz_sender_timer_callback(self):
        
        #calculating the forward kinematics of the camera pose with the values from the robot
        frontcamera_forward_kin_matrix = self.pin_to_front_camera_chain.forward_kinematics(self.received_geom_joint_values) #homogeneous matrix
        homogenous_xyz = frontcamera_forward_kin_matrix[:, 3]
        
        #extrapolating the xyz values
        frontcamera_xyz = [homogenous_xyz[0], homogenous_xyz[1], homogenous_xyz[2]] 
        frontcamera_xyz = np.array(frontcamera_xyz)
        print('forward kinematics position calculated and send to ee_inv_kin_finder')
        print(frontcamera_xyz)

        #publishing the calculated position of the frontcamera
        camera_xyz_msg = Float32MultiArray()
        camera_xyz_msg.data = [0.0]*3


        for i in range(3):
            camera_xyz_msg.data[i] = frontcamera_xyz[i]
        self.frontcamera_xyz_publisher.publish(camera_xyz_msg) #  PUBLISHED VALUES ARE IN M and RAD

        #to plot
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        self.pin_to_front_camera_chain.plot(self.received_geom_joint_values, ax)
        matplotlib.pyplot.show()


def main(args=None):
    rclpy.init(args=args)

    invers_finder = kinematics_processer()
    rclpy.spin(invers_finder)

    invers_finder.destroy_node()
    rclpy.shutdown()