#!/usr/bin/env python3
import rospy
from Classes import Joint_States

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Joint_State_Publisher')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    joints = Joint_States()

    print("The Puzzlebot joint state publisher is Running")
    try:    
        while not rospy.is_shutdown():
            if not joints._last_time:
                joints._last_time = rospy.Time.now()
            else:
                joints.publish_joint_states()
                joints.update_transform()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass