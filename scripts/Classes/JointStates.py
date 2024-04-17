#!/usr/bin/env python3

# Import python libraries
import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import ROS messages
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Import Classes
from .Puzzlebot import Puzzlebot

class Joint_States(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        Puzzlebot.__init__(self)

        # Declare the publish messages
        self.__joints = JointState()
        self.__joints.name = ["leftWheel", "rightWheel"]
        self.__wrp, self.__wlp = 0.0, 0.0

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
        # Get position
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        # Get Velocity
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z
    
    # Publish the joint states (wr, wl)
    def publish_joint_states(self):
        # Set the joint header (time)
        self.__joints.header.stamp = rospy.Time.now()
        # Set the joint positions and velocities
        dt = self._get_dt()
        self.__wrp += (self._v/self._r) + (self._w*self._l / (2*self._r))*dt
        self.__wlp += (self._v/self._r) - (self._w*self._l / (2*self._r))*dt
        self.__joints.position = [self.__wlp, -self.__wrp]
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
        self.__tf.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))
        # Publish the transform
        self.__tf_broadcaster.sendTransform(self.__tf)
