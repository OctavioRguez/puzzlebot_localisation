#!/usr/bin/env python3
import rospy
import numpy as np

class Puzzlebot:
    def __init__(self):
        # Initial puzzlebot parameters
        self._l = rospy.get_param("/puzzlebot/wheelbase", default = 19.0)
        self._r = rospy.get_param("/puzzlebot/wheel_radius", default = 5.0)
        self._last_time = 0.0

    # Get the time difference for dt
    def _get_dt(self):
        current_time = rospy.Time.now()
        self._dt = (current_time - self._last_time).to_sec()
        self._last_time = current_time

    # Wrap to pi function
    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi
