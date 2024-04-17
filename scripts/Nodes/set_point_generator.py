#!/usr/bin/env python3
import rospy
from Classes import Set_Point

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Set_Point_Generator')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    point_Generator = Set_Point()

    print("The Puzzlebot set point generator is Running")
    try:    
        while not rospy.is_shutdown():
            point_Generator.setTrajectory()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass