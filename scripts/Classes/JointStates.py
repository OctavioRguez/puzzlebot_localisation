#!/usr/bin/env python3

# Import python libraries
import rospy
from tf2_ros import TransformBroadcaster

# Import ROS messages
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Import Classes
from .Puzzlebot import Puzzlebot

class Joint_States(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        super().__init__()

        # Declare the publish messages
        self.__joints = JointState()
        self.__joints.name = ["leftWheel", "rightWheel"]

        self.__tf = TransformStamped()
        self.__tf.header.frame_id = "odom"
        self.__tf.child_frame_id = "base_link"

        # Setup the publishers
        self.__tf_broadcaster = TransformBroadcaster()
        self.__joints_pub = rospy.Publisher('/joint_states', JointState, queue_size = 10)

        # Setup the subscribers
        rospy.Subscriber('/odom', Odometry, self.__callback_odom)

    # Callback function for the odometry
    def __callback_odom(self, msg):
        # Position and orientation
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        self._states["theta"] = msg.pose.pose.orientation.z
        # Velocity
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z
    
    # Publish the joint states (wr, wl)
    def publish_joint_states(self):
        # Set the joint header (time)
        self.__joints.header.stamp = rospy.Time.now()
        # Set the joint positions and velocities
        self.__joints.position = [0.0, 0.0]
        self.__joints.velocity = [(self._v/self._r) - (self._w*self._l / (2*self._r)), (self._v/self._r) + (self._w*self._l / (2*self._r))]
        # Publish the joint states
        self.__joints_pub.publish(self.__joints)

    # Update the transform
    def update_transform(self):
        # Set the transform header (time)
        self.__tf.header.stamp = rospy.Time.now()
        # Set the transform translation and rotation
        self.__tf.transform.translation.x = self._states["x"]
        self.__tf.transform.translation.y = self._states["y"]
        self.__tf.transform.rotation.z = self._states["theta"]
        # Publish the transform
        self.__tf_broadcaster.sendTransform(self.__tf)
