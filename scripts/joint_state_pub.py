#!/usr/bin/env python3
import rospy

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Joint_State_Publisher')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    # joint_state_pub = JointStatePublisher()

    print("The Puzzlebot joint state publisher is Running")
    try:    
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass