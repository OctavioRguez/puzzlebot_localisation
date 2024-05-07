#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler

# Import ROS messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point32

# Import Classes
from .Puzzlebot import Puzzlebot

class Localization(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        Puzzlebot.__init__(self)

        # Initial wheel velocities
        self.__wr, self.__wl = 0.0, 0.0

        self.__set_point = np.array([0.0, 0.0])

        # Kalman filter parameters
        self.__x = np.array([0, 0, 0])
        self.__z = np.array([0, 0, 0])
        self.__C = np.eye(3)
        self.__Q = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self.__P = self.__Q
        self.__R = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

        # Declare the publish messagess
        self.__odom = Odometry()
        self.__odom.header.frame_id = "odom"
        self.__odom.child_frame_id = "base_link"

        self.__kalman = Odometry()
        self.__kalman.header.frame_id = "kalman"
        self.__kalman.child_frame_id = "base_link"

        # Publisher for odometry
        self.__odom_pub = rospy.Publisher("/odom", Odometry, queue_size = 10)
        self.__kalman_pub = rospy.Publisher("/kalman", Odometry, queue_size = 10)
        self.__theta_pub = rospy.Publisher("/theta", Float32, queue_size = 10)
        self.__theta_kalman_pub = rospy.Publisher("/theta_kalman", Float32, queue_size = 10)
        
        # Subscribe to wheel encoder topics
        rospy.Subscriber("/wr", Float32, self.__wr_callback)
        rospy.Subscriber("/wl", Float32, self.__wl_callback)
        rospy.Subscriber("/point", Point32, self.__point_callback)

    # Callback for the right wheel velocity
    def __wr_callback(self, msg):
        self.__wr = msg.data

    # Callback for the left wheel velocity
    def __wl_callback(self, msg):
        self.__wl = msg.data
    
    def __point_callback(self, msg):
        self.__set_point = np.array([msg.x, msg.y])

    # Solve the odometry equations
    def update_odometry(self):
        # Get the time step
        dt = self._get_dt()

        # Compute odometry
        self._v = self._r * (self.__wr + self.__wl) / 2.0
        self._w = self._r * (self.__wr - self.__wl) / self._l

        # Update states
        self._states["theta"] = self._wrap_to_Pi(self._states["theta"] + self._w*dt)
        self._states["x"] += self._v * np.cos(self._states["theta"]) * dt
        self._states["y"] += self._v * np.sin(self._states["theta"]) * dt

        # Publish odometry message
        self.__publish_odometry()
        # Publish angle in rad
        self.__theta_pub.publish(self._states["theta"])

    def __publish_odometry(self):
        # Set the header
        self.__odom.header.stamp = rospy.Time.now()

        # Set the position
        self.__odom.pose.pose.position.x = self._states["x"]
        self.__odom.pose.pose.position.y = self._states["y"]

        # Set the orientation
        self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))

        # Set the velocities
        self.__odom.twist.twist.linear.x = self._v
        self.__odom.twist.twist.angular.z = self._w

        # Publish the message
        self.__odom_pub.publish(self.__odom)

    def kalmanFilter(self):
        # Update measurements
        self.__z = np.array([self._states["x"], self._states["y"], self._states["theta"]])

        # Kalman prediction
        dt = self._get_dt()
        q_dot, theta_dot = self.system(5*np.eye(2), self.__x[2], self.__set_point - np.array([self.__x[0], self.__x[1]]))
        self.__x = self.__x + (np.array([q_dot[0], q_dot[1], theta_dot[0]])*dt).T
        self.__P = self.__P + self.__Q

        # Kalman correction
        self.__R = np.dot(np.dot(self.__P, self.__C.T), np.linalg.inv(np.dot(np.dot(self.__C, self.__P), self.__C.T) + self.__Q))
        self.__K = np.dot(np.dot(self.__P, self.__C.T), np.linalg.inv(np.dot(np.dot(self.__C, self.__P), self.__C.T) + self.__R))
        self.__x = self.__x + np.dot(self.__K, (self.__z.T - np.dot(self.__C, self.__x.T))).T
        self.__P = np.dot(np.dot(np.eye(3) - np.dot(self.__K, self.__C), self.__P), (np.eye(3) - np.dot(self.__K, self.__C)).T) + np.dot(np.dot(self.__K, self.__R), self.__K.T)

        # Publish kalman
        self.__kalman.header.stamp = rospy.Time.now()
        # Set the position
        self.__kalman.pose.pose.position.x = self.__x[0]
        self.__kalman.pose.pose.position.y = self.__x[1]
        # Set the orientation
        self.__kalman.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.__x[2]))
        self.__kalman_pub.publish(self.__kalman)
        # Publish angle in rad
        self.__theta_kalman_pub.publish(self.__x[2])
