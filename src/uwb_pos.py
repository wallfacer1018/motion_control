#!/usr/bin/env python

import rospy
from nlink_parser.msg import LinktrackNodeframe2
import numpy as np
import uwb_kalman

class UWBListener:
    def __init__(self):
        self.current_pos = np.zeros(4)  # Variable to store current 2D position
        self.P = np.eye(4)

    def callback(self, data):
        pos = np.array([data.pos_3d[0], data.pos_3d[1]])  # Only use x and y coordinates
        self.current_pos, self.P = uwb_kalman.kalman_filter(self.current_pos, self.P, pos) # update current positon and P matrix


    def start_listener(self):
        rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.callback)
        rospy.spin()
