#! /usr/bin/env python



import rospy
import sys
import math
import numpy as np
import yaml
import json
import socket
import time
import requests

import actionlib
import std_srvs

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import topological_navigation.msg

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


class nav_benchmark(object):
    
    def __init__(self, filename) :
        self.hostname=None       
        self._cov_xx=[]
        self._cov_xy=[]
        self._cov_xw=[]
        self._cov_yy=[]
        self._cov_yw=[]
        self._cov_ww=[]
        
        self.cancel=False
        self.pause=False
        self.robot_pose = None
        self.user_interventions=0
        self._user_lock=False

        rospy.on_shutdown(self._on_node_shutdown)
        
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.total_dist = 0.0        

        topo_map = rospy.get_param('/topological_map_name','none')
        self.hostname= socket.gethostname()
        rospy.loginfo("Benchmarking Navigation in: %s" %self.hostname)        
        print "starting benchmarking in: "
        
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")

        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_cb)
        rospy.Subscriber('/joy_priority', Bool, self.joy_lock_cb)


        
        rospy.Service('/benchmark_pause', SetBool, self.pause_cb)
        wplist = self.open_waypoint_list(filename)
        start_time = rospy.Time.now()
        start_date =  time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime())
        navtasks=0
        while not self.cancel:
            for i in wplist:
                self.navigate_to_waypoint(i)
                #rospy.sleep(0.5)
                while self.pause and not self.cancel:
                    rospy.sleep(0.5)
                if self.cancel:
                    break
                navtasks+=1
                print (rospy.Time.now() - start_time).secs, self.total_dist, navtasks
                
        end_time = rospy.Time.now()
        end_date = time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime())
        opr_time = end_time - start_time
        
        d={}
        d['robot']=self.hostname
        d['scenario']='navigation benchmarks'
        d['navigation_tasks']=navtasks
        d["start_epoch"]=start_time.secs
        d["end_epoch"]=end_time.secs
        d['operation_time']=opr_time.secs
        d['end_date']=end_date
        d['start_date']=start_date
        d['total_dist']=self.total_dist
        d['map']= topo_map
        d['tmap']= topo_map
        d['max_cov_xx'] = float(np.max(np.asarray(self._cov_xx, dtype=np.float32)))
        d['max_cov_xy'] = float(np.max(np.asarray(self._cov_xy, dtype=np.float32)))
        d['max_cov_xw'] = float(np.max(np.asarray(self._cov_xw, dtype=np.float32)))
        d['max_cov_yy'] = float(np.max(np.asarray(self._cov_yy, dtype=np.float32)))
        d['max_cov_yw'] = float(np.max(np.asarray(self._cov_yw, dtype=np.float32)))
        d['max_cov_ww'] = float(np.max(np.asarray(self._cov_ww, dtype=np.float32)))
        d['mean_cov_xx'] = float(np.average(np.asarray(self._cov_xx, dtype=np.float32)))
        d['mean_cov_xy'] = float(np.average(np.asarray(self._cov_xy, dtype=np.float32)))
        d['mean_cov_xw'] = float(np.average(np.asarray(self._cov_xw, dtype=np.float32)))
        d['mean_cov_yy'] = float(np.average(np.asarray(self._cov_yy, dtype=np.float32)))
        d['mean_cov_yw'] = float(np.average(np.asarray(self._cov_yw, dtype=np.float32)))
        d['mean_cov_ww'] = float(np.average(np.asarray(self._cov_ww, dtype=np.float32)))
        d['user_interventions'] = self.user_interventions


        requests.post('https://script.google.com/macros/s/AKfycbxy1ekygUzxROVlPKU_frO2u68cBx7ti3NNVtLzpoOymxSVyjv-/exec', json=d)

        filename= str(rospy.Time.now().secs)
        fh = open(filename, "w")
#        s_output = str(yml)
#        print s_output
#        fh.write(s_output)
        
        jsn = json.dumps(d)
        print jsn

        s_output = str(jsn)
        print s_output
        fh.write(s_output)
        
        fh.close()

        print self.total_dist, start_time, end_time, opr_time

        
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

    
    def robot_pose_cb(self, msg):
        if self.robot_pose:
            pd = math.hypot(self.robot_pose.position.x-msg.position.x, self.robot_pose.position.y-msg.position.y)
            self.total_dist += pd
            
        self.robot_pose = msg    
    
    
    def amcl_pose_cb(self, msg):
        self._cov_xx.append(msg.pose.covariance[0])
        self._cov_xy.append(msg.pose.covariance[1])
        self._cov_xw.append(msg.pose.covariance[5])
        self._cov_yy.append(msg.pose.covariance[7])
        self._cov_yw.append(msg.pose.covariance[11])
        self._cov_ww.append(msg.pose.covariance[35])

    
    def joy_lock_cb(self, msg):
        if msg.data:            
            if not self._user_lock:
                self.user_interventions+=1
        
        self._user_lock=msg.data
    
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
    rospy.init_node('navigation_benchmarking')
    ps = nav_benchmark(sys.argv[1])
    
