#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from .Puzzlebot import Puzzlebot

class Simulation(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot parameters
        super().__init__()

        # Rover states
        self.__states = {"x":0.0, "y":0.0, "theta":0.0}
        self.__v, self.__w = 0.0, 0.0

        # Declare the publish messages
        self.__position = PoseStamped()

        # Setup the publishers
        self.__pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size = 10)
        self.__wr_pub = rospy.Publisher('/wr', Float32, queue_size = 10)
        self.__wl_pub = rospy.Publisher('/wl', Float32, queue_size = 10)

        # Setup the subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.__callback_cmd_vel)

    # Callback for the puzzlebot velocities
    def __callback_cmd_vel(self, msg):
        self.__v = msg.linear.x
        self.__w = msg.angular.z

    # Wrap to pi function
    def __wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi

    # Solve model
    def solve_equations(self):
        # Get the time step
        self._get_dt()
        # Update rover angle
        self.__states["theta"] = self.__wrap_to_Pi(self.__states["theta"] + self.__w*self._dt)
        # Get rover position
        self.__states["x"] += self.__v*np.cos(self.__states["theta"])*self._dt
        self.__states["y"] += self.__v*np.sin(self.__states["theta"])*self._dt

    # Publish the pose (x, y, theta)
    def publish_pose(self):
        self.__position.header.stamp = rospy.Time.now()
        self.__position.pose.position.x = self.__states["x"]
        self.__position.pose.position.y = self.__states["y"]
        self.__position.pose.orientation.z = self.__states["theta"]
        self.__pose_pub.publish(self.__position)

    # Publish the velocities (wr, wl)
    def publish_velocities(self):
        self.__wr_pub.publish((self.__v/self._r) + (self.__w*self._l / (2*self._r)))
        self.__wl_pub.publish((self.__v/self._r) - (self.__w*self._l / (2*self._r)))
