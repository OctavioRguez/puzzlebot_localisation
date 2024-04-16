#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from .Puzzlebot import Puzzlebot

class Localization(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot parameters
        super().__init__()

        # Initial the puzzlebot states
        self.__states = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # Initialize variables
        self.__wr, self.__wl = 0.0, 0.0
        self.__odom = Odometry()
        self.__odom.header.frame_id = "odom"
        self.__odom.child_frame_id = "base_link"

        # Publisher for odometry
        self.__odom_pub = rospy.Publisher("/odom", Odometry, queue_size = 10)
        
        # Subscribe to wheel encoder topics
        rospy.Subscriber("/wr", Float32, self.__wr_callback)
        rospy.Subscriber("/wl", Float32, self.__wl_callback)

    def __wr_callback(self, msg):
        self.__wr = msg.data

    def __wl_callback(self, msg):
        self.__wl = msg.data

    def update_odometry(self):
        # Get the time step
        self._get_dt()
        
        # Compute odometry
        self.__delta_s = self._r * (self.__wr + self.__wl) * self._dt / 2.0
        self.__delta_theta = self._r * (self.__wr - self.__wl) * self._dt / self._l

        # Update pose
        self.__states["x"] += self.__delta_s * np.cos(self.__states["theta"] + self.__delta_theta / 2.0)
        self.__states["y"] += self.__delta_s * np.sin(self.__states["theta"] + self.__delta_theta / 2.0)
        self.__states["theta"] = self._wrap_to_Pi(self.__states["theta"] + self.__delta_theta)
    
    def publish_odometry(self):
        # Publish odometry message
        self.__odom.header.stamp = rospy.Time.now()

        # Set the position
        self.__odom.pose.pose.position.x = self.__states["x"]
        self.__odom.pose.pose.position.y = self.__states["y"]
        self.__odom.pose.pose.orientation.z = self.__states["theta"]

        # Set the velocity
        self.__odom.twist.twist.linear.x = self.__delta_s / self._dt
        self.__odom.twist.twist.angular.z = self.__delta_theta / self._dt

        # Publish the message
        self.__odom_pub.publish(self.__odom)
