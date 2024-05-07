#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

class Puzzlebot:
    def __init__(self):
        # Initial puzzlebot parameters
        self._l = rospy.get_param("/puzzlebot/wheelbase", default = 19.0)
        self._r = rospy.get_param("/puzzlebot/wheel_radius", default = 5.0)
        self._h = rospy.get_param("/puzzlebot/hinge", default = 1.0)
        self._trajectory = rospy.get_param("/puzzlebot/trajectory", default = "square")
        self._last_time = 0.0

        # Puzzlebot states
        self._states = {"x":0.0, "y":0.0, "theta":0.0}
        self._v, self._w = 0.0, 0.0

    # Get the time difference for dt
    def _get_dt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self._last_time).to_sec()
        self._last_time = current_time
        return dt

    # Wrap to pi function
    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    def system(self, kp, theta, err):
        # Control
        D = np.array([
            [(self._r / 2 * np.cos(theta) - self._h*self._r / self._l * np.sin(theta)),
             (self._r / 2 * np.cos(theta) + self._h*self._r / self._l * np.sin(theta))],
            [(self._r / 2 * np.sin(theta) + self._h*self._r / self._l * np.cos(theta)), 
             (self._r / 2 * np.sin(theta) - self._h*self._r / self._l * np.cos(theta))]
        ])
        u = np.dot(np.linalg.inv(D), np.dot(kp, err))
        q_dot = np.dot(D, u)
        theta_dot = np.dot(np.array([[self._r / self._l, -self._r / self._l]]), u)
        return q_dot, theta_dot
