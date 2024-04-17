#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# Import ROS messages
from geometry_msgs.msg import Twist, Polygon
from nav_msgs.msg import Odometry

# Import Classes
from .Puzzlebot import Puzzlebot

class Controller(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        Puzzlebot.__init__(self)

        # Initialize variables
        self.__set_point = []
        self.__kpt, self.__kpr = 0.5, 0.5
        self.__vmax, self.__wmax = 0.2, 0.2
        self.__error = 0.0
        self.__i = 0

        # Declare the publish messagess
        self.__vel = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to the odometry and set_point topics
        rospy.Subscriber("/odom", Odometry, self.__callback_odom)
        rospy.Subscriber("/set_point", Polygon, self.__set_point_callback)

    # Callback function for the odometry
    def __callback_odom(self, msg):
        # Get position
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        # Get Velocity
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z

    # Callback function for the set point
    def __set_point_callback(self, msg):
        self.__set_point = msg.points

    def control(self):
        # Skip if the robot is not moving
        if not self.__set_point or self.__i >= len(self.__set_point):
            return
        
        # Calculate the errors
        thetad = np.arctan2(self.__set_point[self.__i].y - self._states["y"], 
                                self.__set_point[self.__i].x - self._states["x"])
        thetae = (thetad - self._states["theta"])

        err = np.sqrt((self.__set_point[self.__i].x - self._states["x"])**2 + 
                        (self.__set_point[self.__i].y - self._states["y"])**2) if thetae < 0.05 else 0.0
        
        # Calculate the control input
        self.__vel.linear.x = self.__vmax*np.tanh(err * self.__kpt/self.__vmax)
        self.__vel.angular.z = self.__wmax*np.tanh(thetae * self.__kpr/self.__wmax)

        # Check if the robot has reached the set point
        self.__i = self.__i + 1 if err < 0.05 and thetae < 0.05 else self.__i
        
        # Publish the control input
        self.__vel_pub.publish(self.__vel)