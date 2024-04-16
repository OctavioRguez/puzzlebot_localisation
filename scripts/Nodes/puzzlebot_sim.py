#!/usr/bin/env python3
import rospy
from Classes import Simulation

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Simulation')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    puzzlebot = Simulation()

    print("The Puzzlebot simulation is Running")
    try:    
        while not rospy.is_shutdown():
            if not puzzlebot._last_time:
                puzzlebot._last_time = rospy.Time.now()
            else:
                puzzlebot.solve_equations()
                puzzlebot.publish_pose()
                puzzlebot.publish_velocities()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass