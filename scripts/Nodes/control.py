#!/usr/bin/env python3
import rospy
from Classes import Controller

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Controller')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    controlHandler = Controller()

    print("The Puzzlebot control is Running")
    try:    
        while not rospy.is_shutdown():
            if not controlHandler._last_time:
                controlHandler._last_time = rospy.Time.now()
            else:
                controlHandler.control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass