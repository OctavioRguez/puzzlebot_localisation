#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

# Import ROS messages
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import Classes
from .Puzzlebot import Puzzlebot

class Controller(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        super().__init__()

        # Initialize variables
        self.__set_point = []

        # Declare the publish messagess
        self.__vel = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to the odometry and set_point topics
        rospy.Subscriber("/odom", Odometry, self.__callback_odom)
        rospy.Subscriber("/set_point", Float32MultiArray, self.__set_point_callback)

    # Callback function for the odometry
    def __callback_odom(self, msg):
        # Position and orientation
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        self._states["theta"] = msg.pose.pose.orientation.z
        # Velocity
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z

    # Callback function for the set point
    def __set_point_callback(self, msg):
        self.__set_point = msg.data

    # Control function
    def control(self):
        # Control law
        self.__vel.linear.x = 0.5 * np.sqrt((self.__set_point[0] - self._states["x"])**2 + (self.__set_point[1] - self._states["y"])**2)
        self.__vel.angular.z = 0.5 * (np.arctan2(self.__set_point[1] - self._states["y"], self.__set_point[0] - self._states["x"]) - self._states["theta"])

        # Publish the velocity
        self.__vel_pub.publish(self.__vel)
