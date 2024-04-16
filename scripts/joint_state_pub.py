#!/usr/bin/env python3
import rospy
from Classes import Joint_States

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Joint_State_Publisher')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    joint_state_pub = Joint_States()

    print("The Puzzlebot joint state publisher is Running")
    try:    
        while not rospy.is_shutdown():
            joint_state_pub.publish_joint_states()
            joint_state_pub.update_transform()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass