#!/usr/bin/env python

import rospy
from nlink_parser.msg import LinktrackNodeframe2
import numpy as np
from filterpy.kalman import KalmanFilter

class UWBListener:
    def __init__(self):
        # Initialize the Kalman filter for 2D position
        self.kf = KalmanFilter(dim_x=4, dim_z=2)  # 4 states (x, y, vx, vy) and 2 measurements (x, y)
        self.kf.F = np.array([[1, 0, 1, 0],       # State transition matrix
                              [0, 1, 0, 1],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0],       # Measurement function
                              [0, 1, 0, 0]])
        self.kf.P *= 1000.  # Initial covariance matrix
        self.kf.R = np.eye(2) * 5  # Measurement noise
        self.kf.Q = np.eye(4)      # Process noise
        self.current_pos = np.zeros(2)  # Variable to store current 2D position

    def callback(self, data):
        # UWB position data
        z = np.array([data.pos_3d[0], data.pos_3d[1]])  # Only use x and y coordinates
        # Kalman filter prediction
        self.kf.predict()
        # Kalman filter update
        self.kf.update(z)
        # Update current position
        self.current_pos = self.kf.x[:2]
        # rospy.loginfo("Filtered Position: %s", self.current_pos)

    def start_listener(self):
        rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.callback)
        rospy.spin()
