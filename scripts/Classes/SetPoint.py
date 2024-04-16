#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

# Import ROS messages
from std_msgs.msg import Float32MultiArray

# Import Classes
from .Puzzlebot import Puzzlebot

class Set_Point(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        super().__init__()

        # Declare the publish messagess
        self.__points = Float32MultiArray()

        # Publisher for the set point
        self.__vel_pub = rospy.Publisher("/set_point", Float32MultiArray, queue_size = 10)

