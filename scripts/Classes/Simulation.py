#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

# Import ROS messages
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped

# Import Classes
from .Puzzlebot import Puzzlebot

class Simulation(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        super().__init__()

        # Declare the publish messages
        self.__position = PoseStamped()

        # Setup the publishers
        self.__pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size = 10)
        self.__wr_pub = rospy.Publisher("/wr", Float32, queue_size = 10)
        self.__wl_pub = rospy.Publisher("/wl", Float32, queue_size = 10)

        # Setup the subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.__callback_cmd_vel)

    # Callback for the puzzlebot velocities
    def __callback_cmd_vel(self, msg):
        self._v = msg.linear.x
        self._w = msg.angular.z

    # Solve model
    def solve_equations(self):
        # Get the time step
        dt = self._get_dt()
        # Update rover angle
        self._states["theta"] = self._wrap_to_Pi(self._states["theta"] + self._w*dt)
        # Get rover position
        self._states["x"] += self._v*np.cos(self._states["theta"])*dt
        self._states["y"] += self._v*np.sin(self._states["theta"])*dt

    # Publish the pose (x, y, theta)
    def publish_pose(self):
        # Set the position data
        self.__position.header.stamp = rospy.Time.now()
        self.__position.pose.position.x = self._states["x"]
        self.__position.pose.position.y = self._states["y"]
        self.__position.pose.orientation.z = self._states["theta"]
        # Publish the position
        self.__pose_pub.publish(self.__position)

    # Publish the velocities (wr, wl)
    def publish_velocities(self):
        # Calculate the velocities (rad/s) and publish them
        self.__wr_pub.publish((self._v/self._r) + (self._w*self._l / (2*self._r)))
        self.__wl_pub.publish((self._v/self._r) - (self._w*self._l / (2*self._r)))
