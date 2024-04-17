#!/usr/bin/env python3

# Import python libraries
import rospy

# Import ROS messages
from geometry_msgs.msg import Point, Polygon

# Import Classes
from .Puzzlebot import Puzzlebot

class Set_Point(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        super().__init__()

        # Declare the publish messagess
        self.__points = Polygon()

        # Publisher for the set point
        self.__points_pub = rospy.Publisher("/set_point", Polygon, queue_size = 10)
    
    # Points for the square trajectory
    def __square(self):
        self.__points.points = [Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0), Point(0, 0, 0)]
    
    # Points for the pentagon trajectory
    def __pentagon(self):
        self.__points.points = [Point(1, 0, 0), Point(0.5, 1, 0), Point(0, 0.5, 0), Point(0, 0, 0), Point(1, 0, 0)]
    
    # Points for the triangle trajectory
    def __triangle(self):
        self.__points.points = [Point(1, 0, 0), Point(0.5, 0.5, 0), Point(0, 0, 0)]

    # Set the trajectory type
    def setTrajectory(self):
        if self._trajectory == "square":
            self.__square()
        elif self._trajectory == "pentagon":
            self.__pentagon()
        elif self._trajectory == "triangle":
            self.__triangle()
        else:
            rospy.loginfo("Invalid trajectory")
        self.__points_pub.publish(self.__points)
