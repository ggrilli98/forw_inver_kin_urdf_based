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


class kinamtics_furnisher(Node):
    def __init__(self):
        super().__init__('kinematic_node')

        self.kinamtic_furnisher_timer = self.create_timer(4, self.furnisher_callback)

        self.inverse_kin_join_values_publisher = self.create_publisher(
            Float32MultiArray,
            'distance_container_from_the_O0',
            10
        )

        self.i = 0


    def furnisher_callback(self):
        self.i = self.i + 1
        msg = Float32MultiArray()
        msg.data = [0.000, 0.8 , 0.5] #(in meters)
        print(msg.data)
        self.inverse_kin_join_values_publisher.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    
    inverse_publisher = kinamtics_furnisher()
    rclpy.spin(inverse_publisher)

    rclpy.shutdown()