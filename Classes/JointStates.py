#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from .Puzzlebot import Puzzlebot

class Joint_States(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot parameters
        super().__init__()

        # Rover states
        self.__states = {"x":0.0, "y":0.0, "theta":0.0}
        self.__v, self.__w = 0.0, 0.0

        # Declare the publish messages
        self.__joints = JointState()
        self.__joints.name = ["leftWheel", "rightWheel"]

        self.__tf = TransformStamped()
        self.__tf.header.frame_id = "odom"
        self.__tf.child_frame_id = "base_link"

        # Setup the publishers
        self.__tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.__joints_pub = rospy.Publisher('/joint_states', JointState, queue_size = 10)

        # Setup the subscribers
        rospy.Subscriber('/odom', Odometry, self.__callback_odom)

    def __callback_odom(self, msg):
        self.__states["x"] = msg.pose.pose.position.x
        self.__states["y"] = msg.pose.pose.position.y
        self.__states["theta"] = msg.pose.pose.orientation.z
        self.__v = msg.twist.twist.linear.x
        self.__w = msg.twist.twist.angular.z
    
    # Publish the joint states (wr, wl)
    def publish_joint_states(self):
        self.__joints.header.stamp = rospy.Time.now()
        self.__joints.position = [0.0, 0.0]
        self.__joints.velocity = [(self.__v/self._r) - (self.__w*self._l / (2*self._r)), (self.__v/self._r) + (self.__w*self._l / (2*self._r))]
        self.__joints_pub.publish(self.__joints)

    # Update the transform
    def update_transform(self):
        self.__tf.header.stamp = rospy.Time.now()
        self.__tf.transform.translation.x = self.__states["x"]
        self.__tf.transform.translation.y = self.__states["y"]
        self.__tf.transform.rotation.z = self.__states["theta"]
        self.__tf_broadcaster.sendTransform(self.__tf)