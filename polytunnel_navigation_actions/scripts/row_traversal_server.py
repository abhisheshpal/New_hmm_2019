#!/usr/bin/env python

import rospy
#import math
#import numpy as np
import threading
import subprocess

import actionlib
import polytunnel_navigation_actions.msg
#import std_msgs.msg

#from visualization_msgs.msg import MarkerArray

class inRowTravServer(object):

    _feedback = polytunnel_navigation_actions.msg.inrownavFeedback()
    _result   = polytunnel_navigation_actions.msg.inrownavResult()

    def __init__(self, name):
        self.active_goal=None
        self.new_goal = False     
        self.cancelled = False
        self.p={}
        rospy.loginfo("Creating action server.")
        self._as = actionlib.ActionServer('/test_act_serv', polytunnel_navigation_actions.msg.inrownavAction,
                                          self.executeCallback, cancel_cb =  self.preemptCallback, auto_start = False)
        #self._as.register_preempt_callback(self.preemptCallback)
        #self._as.register_cancel_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        rospy.spin()
        

    def start_goal(self, gh):
        gh.set_accepted()
        self.active_goal=gh
        self.cancelled = False
        self.new_goal = False
        x = threading.Thread(target=self.execute_goal, args=(gh,1))
        x.start()



    def executeCallback(self, gh):
        rospy.loginfo("New goal received")
        goal_id = gh.get_goal_id()
        print goal_id.id
        if not self.active_goal:
            
            self.start_goal(gh)
        else:
            #gh.set_accepted()   
            self.new_goal = True
            print "Finishing Active Goal", self.active_goal.get_goal_id().id
            self.cancelled =True
            while self.new_goal:# != None:
                rospy.sleep(0.05)
            print "Starting new goal"
            self.start_goal(gh)
        print "outta here"



    def execute_goal(self, gh, dummy):
        i=0
        while i<10 and not self.cancelled and not self.new_goal:
            print i
            rospy.sleep(0.5)
            i+=1

        print "DONISH"
        print gh.get_goal_id().id
        if not self.cancelled:
            print "Not Cancelled no new goal"#, self.active_goal.get_goal_id().id
            self.active_goal.set_succeeded()
            #self.active_goal=None
        elif self.new_goal:
            print "Cancelled new goal"
            print self.active_goal, self.active_goal.get_goal_id().id
            self.active_goal.set_succeeded()
            print "DONE"
        else:
            print "Cancelled no new goal"
            print self.active_goal.get_goal_id().id
            self.active_goal.set_canceled()
            print "DONE"

        print "just checking"
        self.active_goal=None
        self.new_goal=False        

        print "Thread Finished"



    def preemptCallback(self, gh):
        rospy.loginfo("Row Traversal Cancelled")
        self.cancelled = True
           

if __name__ == '__main__':
    rospy.init_node('row_traversal')
    server = inRowTravServer(rospy.get_name())