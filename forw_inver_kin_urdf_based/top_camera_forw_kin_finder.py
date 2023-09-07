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

        self.joint_geom_values_sub = self.create_subscription(
            Float32MultiArray,
            'calculated_joints_values_inverse_kin',
            self.top_camera_pose_calculator,
            10
        )

        self.topcamera_xyz_publisher = self.create_publisher(
            Float32MultiArray,
            'top_camera_xyz',
            10
        )

        #Definiton of the robot arm links, joint and structure
        self.pin_to_top_camera_chain = ikpy.chain.Chain.from_urdf_file("/home/gregorio/ros2_ws/src/forw_inver_kin_urdf_based/models/pin_to_top_camera.urdf")
        print(self.pin_to_top_camera_chain.links)
        print(self.pin_to_top_camera_chain.name)
        print(self.pin_to_top_camera_chain.active_links_mask)
        print(self.pin_to_top_camera_chain._urdf_metadata)

        self.received_geom_joint_values = [0.0]*7
        self.received_geom_joint_values = np.array(self.received_geom_joint_values)
               

    def top_camera_pose_calculator(self, msg):
   
        # RECEIVED VALUES ARE IN MM AND DEG, THE TRANSFORAMTION WORKS IN M AND RADIANS 
        # self.received_geom_joint_values[0] = 0.0
        # self.received_geom_joint_values[1] = 0.0
        # self.received_geom_joint_values[2] = msg.data[0]/360*2*math.pi      #boom tilt
        # self.received_geom_joint_values[3] = msg.data[1]/1000               #boom extension
        # self.received_geom_joint_values[4] = msg.data[2]/360*2*math.pi      #spreader pitch
        # self.received_geom_joint_values[5] = msg.data[3]/360*2*math.pi      #spreader yaw
        # self.received_geom_joint_values[6] = msg.data[4]/1000               #spreader lateral
        # print('ricevuti',self.received_geom_joint_values)
        print('ricevuto', msg.data)
        self.received_geom_joint_values[0] = msg.data[0]
        self.received_geom_joint_values[1] = msg.data[1]
        self.received_geom_joint_values[2] = msg.data[2]      #boom tilt
        self.received_geom_joint_values[3] = msg.data[3]               #boom extension
        self.received_geom_joint_values[4] = msg.data[4]      #spreader pitch
        self.received_geom_joint_values[5] = msg.data[5]      #spreader yaw
        self.received_geom_joint_values[6] = msg.data[6]

        
        print('usati per trasformazione',self.received_geom_joint_values)
        
        

        
        #calculating the forward kinematics of the camera pose with the values from the robot
        topcamera_forward_kin_matrix = self.pin_to_top_camera_chain.forward_kinematics(self.received_geom_joint_values) #homogeneous matrix
        homogenous_xyz = topcamera_forward_kin_matrix[:, 3]
        
        #extrapolating the xyz values
        topcamera_xyz = [homogenous_xyz[0], homogenous_xyz[1], homogenous_xyz[2]] 
        topcamera_xyz = np.array(topcamera_xyz)
        print('forward kinematics position calculated and sent to ee_inv_kin_finder')
        print(topcamera_xyz)

        #publishing the calculated position of the frontcamera
        top_camera_xyz_msg = Float32MultiArray()
        top_camera_xyz_msg.data = [0.0]*3


        for i in range(3):
            top_camera_xyz_msg.data[i] = topcamera_xyz[i]
        self.topcamera_xyz_publisher.publish(top_camera_xyz_msg) #  PUBLISHED VALUES ARE IN M and RAD

        #to plot
        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        self.pin_to_top_camera_chain.plot(self.received_geom_joint_values, ax)
        matplotlib.pyplot.show()


def main(args=None):
    rclpy.init(args=args)

    invers_finder = kinematics_processer()
    rclpy.spin(invers_finder)

    invers_finder.destroy_node()
    rclpy.shutdown()