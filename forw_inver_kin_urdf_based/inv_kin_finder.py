#!/usr/bin/env python3
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python import get_resource
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from rclpy.qos import QoSProfile
# from apriltag_msgs.msg import AprilTagDetectionArray
# from apriltag_msgs.msg import AprilTagDetection
from std_msgs.msg import Float32MultiArray

import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


class kinematics_processer(Node):
    def __init__(self):
        super().__init__('kinematic_node')

        self.inverse_value_receiver_subscirption = self.create_subscription(
            Float32MultiArray,
            'distance_container_from_the_O0',
            self.distance_values_inverse_callback,
            10
        )

        self.inverse_kin_join_values_publisher = self.create_publisher(
            Float32MultiArray,
            'calculated_joints_values_inverse_kin',
            10
        )

        #Definiton of the robot arm links, joint and structure
        self.my_chain = ikpy.chain.Chain.from_urdf_file("/home/gregorio/ros2_ws/src/forw_inver_kin_urdf_based/models/shoulder_model.urdf")
        print(self.my_chain.links)
        print(self.my_chain.name)
        print(self.my_chain.active_links_mask)
        print(self.my_chain._urdf_metadata)
        self.desired_position_xyz = [0.0]*3
        self.desired_orientation = [0.0, 0.0, 1.0]
        
        #self.desired_position_xyz = np.array(self.desired_position_xyz)

    def distance_values_inverse_callback(self, msg):
    
        for i in range(3):
            self.desired_position_xyz[i] = msg.data[i]
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
        joint_values = Float32MultiArray()
        joint_values.data = [0.0]*5


        for i in range(5):
            joint_values.data[i] = self.inverse_kin_joint_values[i+1]
        self.inverse_kin_join_values_publisher.publish(joint_values) #  PUBLISHED VALUES ARE IN DEG AND METERS

        #to plot

        ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
        self.my_chain.plot(self.inverse_kin_joint_values, ax)
        matplotlib.pyplot.show()


def main(args=None):
    rclpy.init(args=args)

    invers_finder = kinematics_processer()
    rclpy.spin(invers_finder)

    invers_finder.destroy_node()
    rclpy.shutdown()