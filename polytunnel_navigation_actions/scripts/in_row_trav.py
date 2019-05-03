#!/usr/bin/env python

import rospy
import math

import numpy as np

import tf

import actionlib
import polytunnel_navigation_actions.msg
import std_msgs.msg

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from strands_navigation_msgs.msg import TopologicalMap



class inRowTravServer(object):

    _feedback = polytunnel_navigation_actions.msg.inrownavFeedback()
    _result   = polytunnel_navigation_actions.msg.inrownavResult()

    def __init__(self, name):
        self.cancelled = False
        self._action_name = name
        self._got_top_map=False
        self.lnodes=None

        while not self.lnodes:
            rospy.loginfo("Waiting for topological map")
            rospy.Subscriber('/topological_map', TopologicalMap, self.topological_map_cb)
            if not self.lnodes:
                rospy.sleep(1.0)
        
        
        rospy.Subscriber('/closest_node', std_msgs.msg.String, self.closest_node_cb)
        self.ppub = rospy.Publisher('/row_line', Path, queue_size=1)


        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, polytunnel_navigation_actions.msg.inrownavAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.spin()


    def topological_map_cb(self, msg):
        self.lnodes=msg


    def closest_node_cb(self, msg):
        if self.closest_node_cb != msg.data:
            self.closest_node=msg.data
            print self.closest_node


    def get_node_position(self, node):
        pose=None
        for i in self.lnodes.nodes:
            if i.name == node:
                pose = i.pose
                break
        return pose


    def _distance_between_poses(self, posea, poseb):
        return math.hypot(poseb.position.x-posea.position.x, poseb.position.y-posea.position.y), math.atan2(poseb.position.y-posea.position.y, poseb.position.x-posea.position.x)


    def _get_path(self, posea, poseb):
        the_path = Path()
        the_path.header.frame_id='map'
        radius, ang = self._distance_between_poses(posea, poseb)
        the_quat = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, ang))
        
        for i in np.arange(0, radius, 0.5):
            d=PoseStamped()
            d.header.frame_id='map'
            d.pose.position.x=posea.position.x + i*np.cos(ang)
            d.pose.position.y=posea.position.y + i*np.sin(ang)
            d.pose.orientation = the_quat
            the_path.poses.append(d)
        
        print the_path
        return the_path
        

    def executeCallback(self, goal):
        initial_pose=self.get_node_position(self.closest_node)        
        final_pose=goal.target_pose.pose
        self.ppub.publish(self._get_path(initial_pose, final_pose))

        self._as.set_succeeded()


    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node('row_traversal')
    server = inRowTravServer(rospy.get_name())