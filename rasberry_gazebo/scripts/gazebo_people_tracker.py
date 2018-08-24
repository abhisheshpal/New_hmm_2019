#!/usr/bin/env python

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Vector3
from bayes_people_tracker.msg import PeopleTracker
import numpy as np


class track_actors(object):
    """
    """
    def __init__(self, robot_names, actor_names):
        """
        """
        self.listener = tf.TransformListener()
        
        self.actor_poses = {}
        self.actor_pose_subs = {}
        self.actor_names = actor_names
        for actor in self.actor_names:
            self.actor_poses[actor] = PoseWithCovarianceStamped()
            self.actor_pose_subs[actor] = rospy.Subscriber('/%s/amcl_pose' %(actor), PoseWithCovarianceStamped , self.actor_pose_callback, callback_args=actor)
            
        self.robot_poses = {}
        self.robot_pose_subs = {}
        self.robot_people_tracker_msgs = {}
        self.robot_people_tracker_pubs = {}
        self.robot_names = robot_names
        for robot in self.robot_names:
            self.robot_poses[robot] = PoseWithCovarianceStamped()
            self.robot_people_tracker_msgs[robot] = PeopleTracker()
            
            if len(robot) == 0: # if the robot has no prefixed name     
                input_topic = '/amcl_pose'
                output_topic = '/people_tracker/positions'
            else:
                input_topic = '/%s/amcl_pose' %(robot)
                output_topic = '/%s/people_tracker/positions' %(robot)
                
            self.robot_pose_subs[robot] = rospy.Subscriber(input_topic, PoseWithCovarianceStamped , self.robot_pose_callback, callback_args=robot)
            self.robot_people_tracker_pubs[robot] = rospy.Publisher(output_topic, PeopleTracker, queue_size=10)
        
    def actor_pose_callback(self, msg, actor):
        self.actor_poses[actor].pose.pose.position.x = msg.pose.pose.position.x
        self.actor_poses[actor].pose.pose.position.y = msg.pose.pose.position.y
        self.actor_poses[actor].pose.pose.position.z = msg.pose.pose.position.z
        self.actor_poses[actor].pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.actor_poses[actor].pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.actor_poses[actor].pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.actor_poses[actor].pose.pose.orientation.w = msg.pose.pose.orientation.w
    
    def robot_pose_callback(self, msg, robot):
        self.robot_poses[robot].header = msg.header
        self.robot_poses[robot].pose.pose.position.x = msg.pose.pose.position.x
        self.robot_poses[robot].pose.pose.position.y = msg.pose.pose.position.y
        self.robot_poses[robot].pose.pose.position.z = msg.pose.pose.position.z
        self.robot_poses[robot].pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.robot_poses[robot].pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.robot_poses[robot].pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.robot_poses[robot].pose.pose.orientation.w = msg.pose.pose.orientation.w
    
    def run(self):
        while not rospy.is_shutdown():
            self.publish_people_tracker_topics()
            rospy.sleep(0.25)

    def publish_people_tracker_topics(self):
        for robot in self.robot_names:       
                        
            people_tracker = self.robot_people_tracker_msgs[robot]
            publisher = self.robot_people_tracker_pubs[robot]

            people_tracker.header = self.robot_poses[robot].header            
            
            actor_poses = []
            actor_velocities_list = []
            actor_distances = []
            actor_angles = []
            for actor in self.actor_names:

                distance, angle, lin_vel = self.compute_polar_coords(robot, actor)
                
                actor_velocities = Vector3()
                actor_velocities.x = lin_vel[0]
                actor_velocities.y = lin_vel[1]
                actor_velocities.z = lin_vel[2]    
                
                actor_poses.append(self.actor_poses[actor].pose.pose)
                actor_velocities_list.append(actor_velocities)
                actor_distances.append(distance)
                actor_angles.append(angle)
            
            people_tracker.uuids = self.actor_names
            people_tracker.poses = actor_poses 
            people_tracker.velocities = actor_velocities_list 
            people_tracker.distances = actor_distances 
            people_tracker.angles = actor_angles
            people_tracker.min_distance = np.min(actor_distances)
            people_tracker.min_distance_angle = actor_angles[np.argmin(actor_distances)]
            
            publisher.publish(people_tracker)
            
            
    def compute_polar_coords(self, robot, actor):
        
        if len(robot) == 0: # if the robot has no prefixed name     
            robot_frame = "/base_link"
        else:
            robot_frame = "/"+robot+"/base_link"
            
        self.listener.waitForTransform("/map", "/"+actor+"/base_link", rospy.Time(0), rospy.Duration(1.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/map", "/"+actor+"/base_link", now, rospy.Duration(1.0))
            (lin_vel, ang_vel) = self.listener.lookupTwist("/map", "/"+actor+"/base_link", rospy.Time(0), rospy.Duration(0.5))
            #info = "Transform between /map and /" + actor + " received successfully."
            #rospy.loginfo(info)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            warning = "Transform between /map and /" + actor + " may not exist. Setting " + actor + " velocities to zero."  
            rospy.logwarn(warning)
            lin_vel = (0.0, 0.0, 0.0)
        
        self.listener.waitForTransform(robot_frame, "/"+actor+"/base_link", rospy.Time(0), rospy.Duration(4.0))
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(robot_frame, "/"+actor+"/base_link", now, rospy.Duration(4.0))
            (trans,rot) = self.listener.lookupTransform(robot_frame, "/"+actor+"/base_link", now)
            #info = "Transform between " + robot_frame + " and /" + actor + "/base_link received successfully."              
            #rospy.loginfo(info)            
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            warning = "Transform between " + robot_frame + " and /" + actor + "/base_link may not exist"
            rospy.logwarn(warning)
        
        distance = np.sqrt(np.sum(np.array(trans[:2])**2)) # not taking z component into account?
        angle = np.arccos(trans[0]/distance) # angle wrt the robots coordinate frame?
        
        return distance, angle, lin_vel # linear (rather that angular) are the correct velocities?
                  
                
            
            
if __name__ == "__main__":
    try:
        
        robot_names = rospy.get_param('gazebo_people_tracker/robot_names')
        actor_names = rospy.get_param('gazebo_people_tracker/actor_names')
        
        rospy.init_node('gazebo_people_tracker')
        rospy.sleep(1e-3) 
        rate=rospy.Rate(30)
    
        track_actors_obj = track_actors(robot_names, actor_names)
        track_actors_obj.run()
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass        
