#!/usr/bin/env python3
import rospy
from Classes import Localization

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Localisation')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    puzzlebot = Localization()

    print("The Puzzlebot localisation is Running")
    try:    
        while not rospy.is_shutdown():
            if not puzzlebot._last_time:
                puzzlebot._last_time = rospy.Time.now()
            else:
                puzzlebot.update_odometry()
                puzzlebot.publish_odometry()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass