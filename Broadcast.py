#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TfBroadcasterNode:
    def __init__(self):
        rospy.init_node('tf_broadcaster')
        
        # Initialize a tf2_ros.TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the transform message
        self.transform = TransformStamped()
        self.transform.header.frame_id = "odom"
        self.transform.child_frame_id = "base_link"

        # Update rate (10 Hz)
        self.rate = rospy.Rate(10)

    def update_transform(self):
        # Set the timestamp
        self.transform.header.stamp = rospy.Time.now()

        # Set the translation
        self.transform.transform.translation.x = 0.0  # Example translation value, adjust as needed
        self.transform.transform.translation.y = 0.0  # Example translation value, adjust as needed
        self.transform.transform.translation.z = 0.0  # Example translation value, adjust as needed

        # Set the rotation (quaternion)
        self.transform.transform.rotation.x = 0.0  # Example rotation value, adjust as needed
        self.transform.transform.rotation.y = 0.0  # Example rotation value, adjust as needed
        self.transform.transform.rotation.z = 0.0  # Example rotation value, adjust as needed
        self.transform.transform.rotation.w = 1.0  # Example rotation value, adjust as needed

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.transform)

    def run(self):
        while not rospy.is_shutdown():
            self.update_transform()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tf_node = TfBroadcasterNode()
        tf_node.run()
    except rospy.ROSInterruptException:
        pass
