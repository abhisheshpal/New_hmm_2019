#!/usr/bin/env python

import rospy
#import math
#import numpy as np

import actionlib
import polytunnel_navigation_actions.msg
#import std_msgs.msg

#from visualization_msgs.msg import MarkerArray

class inRowTravServer(object):

    _feedback = polytunnel_navigation_actions.msg.inrownavFeedback()
    _result   = polytunnel_navigation_actions.msg.inrownavResult()

    def __init__(self, name):

        rospy.loginfo("Creating action server.")
        self._as = actionlib.ActionServer('/test_act_serv', polytunnel_navigation_actions.msg.inrownavAction, self.executeCallback, self.preemptCallback, auto_start = False)
        
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.spin()



    def executeCallback(self, goal):
        rospy.loginfo("New goal received")
        print goal
        self.cancelled = False
        

    def preemptCallback(self):
        rospy.loginfo("Row Traversal Cancelled")
        self.cancelled = True
#        self._send_velocity_commands(0.0, 0.0, 0.0)
#        self.backwards_mode=False


if __name__ == '__main__':
    rospy.init_node('row_traversal')
    server = inRowTravServer(rospy.get_name())