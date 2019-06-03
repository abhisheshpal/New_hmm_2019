#! /usr/bin/env python

import rospy
import sys

import std_srvs
# Brings in the SimpleActionClient
from std_srvs.srv import SetBool
import actionlib
import topological_navigation.msg
import yaml


class topol_nav_patrol(object):
    
    def __init__(self, filename) :
        self.cancel=False
        self.pause=False
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")


        rospy.Service('/patroller_pause', SetBool, self.pause_cb)
        wplist = self.open_waypoint_list(filename)
        
        while not self.cancel:
            for i in wplist:
                self.navigate_to_waypoint(i)
                rospy.sleep(0.5)
                while self.pause:
                    rospy.sleep(0.5)
                if self.cancel:
                    break
        
    def pause_cb(self, req):
        if not req.data:
            self.pause=True
            ans = std_srvs.srv.SetBoolResponse()
            ans.success = True
            ans.message = 'Patrolling paused'
            self.client.cancel_all_goals()
        else:
            self.pause=False
            ans = std_srvs.srv.SetBoolResponse()
            ans.success = True
            ans.message = 'Patrolling activated'
            

        return ans

    
    def open_waypoint_list(self, filename):
                
        with open(filename, 'r') as listf:
            try:
                wplist = yaml.safe_load(listf)
            except yaml.YAMLError as exc:
                print(exc)
        return wplist

    def navigate_to_waypoint(self, goal):
        navgoal = topological_navigation.msg.GotoNodeGoal()
    
        print "Requesting Navigation to %s" %goal
    
        navgoal.target = goal
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps
        

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        self.cancel=True
        #sleep(2)


if __name__ == '__main__':
    print 'Argument List:',str(sys.argv)
    if len(sys.argv) < 2 :
	sys.exit(2)
    rospy.init_node('topol_nav_test')
    ps = topol_nav_patrol(sys.argv[1])
    
