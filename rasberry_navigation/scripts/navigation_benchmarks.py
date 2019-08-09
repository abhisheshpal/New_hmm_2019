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
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
import topological_navigation.msg
from sensor_msgs.msg import LaserScan


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
        self.help_requested=0
        self._user_lock=False
        self.min_scan_range=100.0
        self.dist2obs=[]


        rospy.on_shutdown(self._on_node_shutdown)
        
        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)
        self.total_dist = 0.0        

        topo_map = rospy.get_param('/topological_map_name','none')
        self.hostname= socket.gethostname()
        rospy.loginfo("Benchmarking Navigation in: %s" %self.hostname)        
        print "starting benchmarking in: "
        
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")

        rospy.Subscriber('/row_traversal/notification', String, self.row_traversal_not_cb)
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_cb)
        rospy.Subscriber('/teleop_joy/joy_priority', Bool, self.joy_lock_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        
        rospy.Service('/benchmark_pause', SetBool, self.pause_cb)
        wplist = self.open_waypoint_list(filename)
        start_time = time.time()
        start_date =  time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime())
        navtasks=0
        while not self.cancel:
#            if not self.cancel:
#                self.home_wheels()
#                rospy.sleep(10.0)
            
            for i in wplist:
                self.navigate_to_waypoint(i)
                #rospy.sleep(0.5)
                while self.pause and not self.cancel:
                    rospy.sleep(0.5)
                if self.cancel:
                    break
                navtasks+=1
                print time.time() - start_time, self.total_dist, navtasks
            
        end_time = time.time()
        end_date = time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime())
        opr_time = end_time - start_time
        
        d={}
        d['robot']=self.hostname
        d['scenario']='navigation benchmarks'
        d["start_epoch"]=start_time
        d["end_epoch"]=end_time
        d['operation_time']=opr_time
        d['start_date']=start_date
        d['total_dist']=self.total_dist
        d['average_speed']= self.total_dist/opr_time
        d['tmap']= topo_map
        d['user_interventions'] = self.user_interventions
        d['help_requested'] = self.help_requested        
        d['navigation_tasks']=navtasks
        d['end_date']=end_date
        d['map']= topo_map
        d['min_dist_to_obs'] = self.min_scan_range
        d['mean_dist_to_obs'] = float(np.average(np.asarray(self.dist2obs, dtype=np.float32)))
        d['max_cov_xx'] = float(np.max(np.asarray(self._cov_xx, dtype=np.float32)))
        d['max_cov_xy'] = float(np.max(np.asarray(self._cov_xy, dtype=np.float32)))
        d['max_cov_yy'] = float(np.max(np.asarray(self._cov_yy, dtype=np.float32)))
        d['max_cov_ww'] = float(np.max(np.asarray(self._cov_ww, dtype=np.float32)))
        d['mean_cov_xx'] = float(np.average(np.asarray(self._cov_xx, dtype=np.float32)))
        d['mean_cov_xy'] = float(np.average(np.asarray(self._cov_xy, dtype=np.float32)))
        d['mean_cov_yy'] = float(np.average(np.asarray(self._cov_yy, dtype=np.float32)))
        d['mean_cov_ww'] = float(np.average(np.asarray(self._cov_ww, dtype=np.float32)))
        


        requests.post('https://script.google.com/macros/s/AKfycbxy1ekygUzxROVlPKU_frO2u68cBx7ti3NNVtLzpoOymxSVyjv-/exec', json=d)





        filename= str(int(time.time()))+'.json'
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



    def home_wheels(self):
        rospy.wait_for_service('/base_driver/home_steering')
        try:
            activate_service = rospy.ServiceProxy('/base_driver/home_steering', Trigger)
            resp1 = activate_service()
            print resp1.message
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e      

    def row_traversal_not_cb(self, msg):
        self.help_requested+=1
        
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
        self._cov_yy.append(msg.pose.covariance[7])
        self._cov_ww.append(msg.pose.covariance[35])

        print msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[7], msg.pose.covariance[35]

    
    def scan_cb(self, msg):
        min_range = min(x for x in msg.ranges if x > msg.range_min)
        self.min_scan_range=min(self.min_scan_range, min_range)
        self.dist2obs.append(min_range)
        

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
    
