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
from apriltag_hafola.apriltag_msgs.msg import AprilTagDetectionArray #to check why not working, it should

class kinematics_processer(Node):
    def __init__(self):
        super().__init__('kinematic_node')

        self.inverse_kin_join_values_publisher = self.create_publisher(
            Float32MultiArray,
            'calculated_joints_values_inverse_kin',
            10
        )


        # subscribers for the front low camera
        self.front_camera_apriltag_distance_xyz_sub  = self.create_subscription(
            Float32MultiArray,
            'front_camera_apriltag_xyz',
            self.front_camera_apriltag_xyz_callback,
            10
        )
        
        self.front_camera_xyz_sub = self.create_subscription(
            Float32MultiArray,
            'front_camera_xyz',
            self.front_camera_xyz_callback,
            10
        )

        # subscribers for the top high camera
        self.fake_front_camera_apriltag_distance_xyz_sub  = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.fake_front_camera_apriltag_xyz_callback,
            10
        )

        self.top_camera_xyz_sub = self.create_subscription(
            Float32MultiArray,
            'top_camera_xyz',
            self.top_camera_xyz_callback,
            10
        )

        

        #Definiton of the robot arm links, joint and structure
        self.my_chain = ikpy.chain.Chain.from_urdf_file("/home/hafola/ros2_ws/src/forw_inver_kin_urdf_based/models/pin_to_ee_model.urdf")
        print(self.my_chain.links)
        print(self.my_chain.name)
        print(self.my_chain.active_links_mask)
        print(self.my_chain._urdf_metadata)
        self.desired_position_xyz = [0.0]*3
        self.desired_orientation = [0.0, 0.0, 1.0]

        self.vector_front_camera_to_apriltag = [-0.035, 0.520, 0.420]
        self.vector_top_camera_to_apriltag = [0.0]
        self.vector_pin_to_front_camera_xyz = [0.0]*3
        self.vector_pin_to_top_camera_xyz = [0.0]*3
        self.vector_pin_to_front_camera_apriltag_xyz = [0.0]*3
        self.vector_pin_to_top_camera_apriltag_xyz = [0.0]*3     

           

        
    # FRONT CAMERA ----------------------------------------------------
        #finds the xyz position of the front camera in the pin frame coordinate frame
    def front_camera_xyz_callback(self, msg):
        
        self.vector_pin_to_front_camera_xyz[0] = msg.data[0]
        self.vector_pin_to_front_camera_xyz[1] = msg.data[1]
        self.vector_pin_to_front_camera_xyz[2] = msg.data[2]    
        print('frontcamera postion', self.vector_pin_to_front_camera_xyz)    

        #calculating the distance between the pin and the aprilag xyz
        self.vector_pin_to_front_camera_apriltag_xyz = np.array(self.vector_pin_to_front_camera_xyz) + np.array(self.vector_front_camera_to_apriltag)
        print('distance pin apriltag', self.vector_pin_to_front_camera_apriltag_xyz)

        null = self.inverse_kin_pin_to_ee_finder(self.vector_pin_to_front_camera_apriltag_xyz)

        #storing the distance between the camera and the identified apriltag
    def front_camera_apriltag_xyz_callback(self, msg):

        self.vector_front_camera_to_apriltag[0] = msg.data[0]/1000 #message in is mm, we want meter for the inverse calculations
        self.vector_front_camera_to_apriltag[1] = msg.data[1]/1000
        self.vector_front_camera_to_apriltag[2] = msg.data[2]/1000
        print('detected apriltag distance', self.vector_top_camera_to_apriltag, 'from front camera')
    #------------------------------------------------------------------



    # TOP CAMERA NOT IMPLEMENTED------------------------------------------------------
        #finds the xyz position of the front camera in the pin frame coordinate frame
    def top_camera_xyz_callback(self, msg):
        
        self.vector_pin_to_top_camera_xyz[0] = msg.data[0]
        self.vector_pin_to_top_camera_xyz[1] = msg.data[1]
        self.vector_pin_to_top_camera_xyz[2] = msg.data[2]        

        #calculating the distance between the pin and the aprilag xyz
        self.vector_pin_to_top_camera_apriltag_xyz = np.array(self.vector_pin_to_top_camera_xyz) + np.array(self.vector_top_camera_to_apriltag)

        #storing the distance between the camera and the identified apriltag
    def fake_front_camera_apriltag_xyz_callback(self, msg):

        print(msg)
        print(msg.pose)
        print(msg.pose.position)
        print(msg.pose.orientation)

        # self.vector_top_camera_to_apriltag[0] = msg.data[0]/1000 #message in is mm, we want meter for the inverse calculations
        # self.vector_top_camera_to_apriltag[1] = msg.data[1]/1000
        # self.vector_top_camera_to_apriltag[2] = msg.data[2]/1000
    #------------------------------------------------------------------




    def inverse_kin_pin_to_ee_finder(self, distance_to_pin_xyz):
    
        for i in range(3):
            self.desired_position_xyz[i] = distance_to_pin_xyz[i]
        print(self.desired_position_xyz)

        self.inverse_kin_joint_values = self.my_chain.inverse_kinematics(self.desired_position_xyz, self.desired_orientation, "Z")
        # self.inverse_kin_joint_values = self.my_chain.inverse_kinematics(self.desired_position_xyz)
        print('inverse kin joint values calculated')
        print(self.inverse_kin_joint_values)

        #checking for error with the inverse kinematics
        #calculating the forward kinamtics with the values from the inverse kinamtics
        forward_kin_check_matrix = self.my_chain.forward_kinematics(self.inverse_kin_joint_values) #homogeneous matrix
        forward_kin_check_homogeneous = forward_kin_check_matrix[:, 3]
        # print(forward_kin_check_matrix)
        #extrapolating the xyz values
        forward_kin_check_xyz = [forward_kin_check_homogeneous[0], forward_kin_check_homogeneous[1], forward_kin_check_homogeneous[2]] 
        forward_kin_check_xyz = np.array(forward_kin_check_xyz)
        print('forward kinematics position caulated')
        print(forward_kin_check_xyz)
        #calculating the error
        error = [self.desired_position_xyz - forward_kin_check_xyz]
        print('total error')
        print(error)

        #publishing the calculated values for the joints
        joint_values_msg = Float32MultiArray()
        joint_values_msg.data = [0.0]*7


        for i in range(7):
            joint_values_msg.data[i] = self.inverse_kin_joint_values[i]
        self.inverse_kin_join_values_publisher.publish(joint_values_msg)  #  PUBLISHED VALUES ARE IN DEG AND METERS
        print('pubblicato come valori', joint_values_msg )

        #to plot

        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        self.my_chain.plot(self.inverse_kin_joint_values, ax)
        matplotlib.pyplot.show()

        return()


def main(args=None):
    rclpy.init(args=args)

    invers_finder = kinematics_processer()
    rclpy.spin(invers_finder)

    invers_finder.destroy_node()
    rclpy.shutdown()