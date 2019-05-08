#!/usr/bin/env python

import rospy
import math
import PyKDL

import numpy as np

import tf

import actionlib
import polytunnel_navigation_actions.msg
import std_msgs.msg


from dynamic_reconfigure.server import Server
from polytunnel_navigation_actions.cfg import RowTraversalConfig

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from strands_navigation_msgs.msg import TopologicalMap

from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

class inRowTravServer(object):

    _feedback = polytunnel_navigation_actions.msg.inrownavFeedback()
    _result   = polytunnel_navigation_actions.msg.inrownavResult()

    def __init__(self, name):

        self.kp_ang_ro= 0.6                     # Proportional gain for initial orientation target
        self.initial_heading_tolerance= 0.005   # Initial heading tolerance [rads]
        self.kp_ang= 0.2                        # Proportional gain for heading correction
        self.kp_y= 0.1                          # Proportional gain for sideways corrections
        self.granularity= 0.5                   # Distance between minigoals along path (carrot points)  
        self.forward_speed= 0.8                 
        
        self.config={}
        
        self.cancelled = False
        self._action_name = name
        self._got_top_map=False
        self.lnodes=None
        self.backwards_mode=False
        self.safety_marker=None

        while not self.lnodes:
            rospy.loginfo("Waiting for topological map")
            rospy.Subscriber('/topological_map', TopologicalMap, self.topological_map_cb)
            if not self.lnodes:
                rospy.sleep(1.0)
        
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/closest_node', std_msgs.msg.String, self.closest_node_cb)
        self.ppub = rospy.Publisher('/row_traversal/row_line', Path, queue_size=1)
        self.cmd_pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.ref_pub = rospy.Publisher('/row_traversal/goal_reference', PoseStamped, queue_size=1)
        self.safety_zone_vis_pub = rospy.Publisher('/row_traversal/safety_zone', Marker, queue_size=1)


        self.dyn_reconf_srv = Server(RowTraversalConfig, self.dyn_reconf_callback)

        #tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        #tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._tf_listerner = tf.TransformListener()
        rospy.loginfo("Creating safety zone.")
        self.define_safety_zone(0.20)


        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, polytunnel_navigation_actions.msg.inrownavAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        self.safety_zone_vis_pub.publish(self.safety_marker)
        
        rospy.spin()


    def dyn_reconf_callback(self, config, level):
        rospy.loginfo("reconfigure request")
        print config, level
        
        if self.config:
            changed_dict = {x: self.config[x] != config[x] for x in self.config if x in config}
            lk = [key  for (key, value) in changed_dict.items() if value]
            print "config changed ", lk[0], config[lk[0]]
            if hasattr(self, lk[0]):
                setattr(self, lk[0], config[lk[0]])
                print lk[0], getattr(self, lk[0])
            #self.set_config(lk[0], config[lk[0]])
            self.config = config
        else:
            print "First config: "#, config.items()            
            self.config = config
            for i in config.items():
                if hasattr(self, i[0]):
                    setattr(self, i[0], i[1])
                    print i[0], getattr(self, i[0])
                    
        return config


    def define_safety_zone(self, clearance):
        base_points=[]
                
        cp0=PointStamped()
        cp0.header.frame_id='top0'
        cp0.point.x=0.0+clearance
        cp0.point.y=0.0+clearance
        cp0.point.z=-0.3
        cp1=PointStamped()
        cp1.header.frame_id='top1'
        cp1.point.x=0.0+clearance
        cp1.point.y=0.0-clearance
        cp1.point.z=-0.3
        cp2=PointStamped()
        cp2.header.frame_id='top2'
        cp2.point.x=0.0+clearance
        cp2.point.y=0.0+clearance
        cp2.point.z=-0.3
        cp3=PointStamped()
        cp3.header.frame_id='top3'
        cp3.point.x=0.0+clearance
        cp3.point.y=0.0-clearance
        cp3.point.z=-0.3

        self._tf_listerner.waitForTransform('base_link','top0',rospy.Time.now(), rospy.Duration(1.0))
        a=self._tf_listerner.transformPoint('base_link', cp0)
        base_points.append(a)
        b=self._tf_listerner.transformPoint('base_link',cp1)
        base_points.append(b)
        c=self._tf_listerner.transformPoint('base_link',cp2)
        base_points.append(c)
        d=self._tf_listerner.transformPoint('base_link',cp3)
        base_points.append(d)
                       
        base_pose = Pose()
        base_pose.position.x=0.0
        base_pose.position.y=0.0
        base_pose.position.z=0.0
        base_pose.orientation.x=0.0
        base_pose.orientation.y=0.0
        base_pose.orientation.z=0.0
        base_pose.orientation.w=1.0

        amarker = Marker()
        amarker.header.frame_id = "base_link"
        amarker.header.stamp = rospy.Time.now()
        amarker.type = 4
        amarker.pose = Pose()
        amarker.scale.x = 0.05
        amarker.color.a = 0.5
        amarker.color.r = 0.9
        amarker.color.g = 0.1
        amarker.color.b = 0.1
        amarker.lifetime = rospy.Duration(0.0)
        amarker.frame_locked = True
        for i in base_points:
            amarker.points.append(i.point)
        amarker.points.append(base_points[0].point)
               
        self.safety_marker=amarker


        


    def topological_map_cb(self, msg):
        self.lnodes=msg


    def robot_pose_cb(self, msg):
        self.robot_pose = msg


    def closest_node_cb(self, msg):
        if self.closest_node_cb != msg.data:
            self.closest_node=msg.data
            print self.closest_node
            #for some stupid reason I need to republish this here
            if self.safety_marker:
                self.safety_zone_vis_pub.publish(self.safety_marker)



    def get_node_position(self, node):
        pose=None
        for i in self.lnodes.nodes:
            if i.name == node:
                pose = i.pose
                break
        return pose


    def _distance_between_poses(self, posea, poseb):
        ang = math.atan2(poseb.position.y-posea.position.y, poseb.position.x-posea.position.x)
        the_quat = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, ang))
        return math.hypot(poseb.position.x-posea.position.x, poseb.position.y-posea.position.y), the_quat, ang



    def _get_angle_between_quats(self, ori1, ori2):
        q1 = PyKDL.Rotation.Quaternion(ori1.x, ori1.y, ori1.z, ori1.w)
        q2 = PyKDL.Rotation.Quaternion(ori2.x, ori2.y, ori2.z, ori2.w)

        ang1 = q1.GetRPY()
        ang2 = q2.GetRPY()
        
        return ang2[2]-ang1[2]

    def _get_path(self, posea, poseb):
        the_path = Path()
        the_path.header.frame_id='map'
        radius, the_quat, ang = self._distance_between_poses(posea, poseb)
        
        
        for i in np.arange(0, radius, self.granularity):
            d=PoseStamped()
            d.header.frame_id='map'
            #d.header.stamp = rospy.Time.now()
            d.pose.position.x=posea.position.x + i*np.cos(ang)
            d.pose.position.y=posea.position.y + i*np.sin(ang)
            d.pose.orientation = the_quat
            the_path.poses.append(d)
        
        d=PoseStamped()
        d.header.frame_id='map'
        #d.header.stamp = rospy.Time.now()
        d.pose.position.x=posea.position.x + radius*np.cos(ang)
        d.pose.position.y=posea.position.y + radius*np.sin(ang)
        d.pose.orientation = the_quat
        the_path.poses.append(d)
        
        
        return the_path
        

    def align_orientation(self, path_to_goal):
        dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[0])
        print ang_diff, math.degrees(ang_diff)
        if ang_diff > (math.pi/2.0) or ang_diff< -(math.pi/2.0):
            self.backwards_mode=True
            print("we should be going backwards")

        else:
            self.backwards_mode=False
            print("forwards heading")

        while np.abs(ang_diff) >= self.initial_heading_tolerance and not self.cancelled:
            self._send_velocity_commands(0.0, 0.0, self.kp_ang_ro*ang_diff)
            rospy.sleep(0.05)
            dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[0])

        self._send_velocity_commands(0.0, 0.0, 0.0)

        print "Done: ", ang_diff
        

    def go_forwards(self, path_to_goal, start_goal):
        
        if self.backwards_mode:
            speed = -self.forward_speed
        else:
            speed = self.forward_speed
        for i in range(start_goal, len(path_to_goal.poses)):
            dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[i])         
            while np.abs(dist)>0.1 and not self.cancelled:
                self._send_velocity_commands(speed, self.kp_y*y_err, self.kp_ang*ang_diff)
                rospy.sleep(0.05)
                self._get_vector_to_pose(path_to_goal.poses[i])

                dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[i])
            if not self.cancelled:
                print("Next Goal")
            else:
                break

            self._send_velocity_commands(0.0, 0.0, 0.0)

        
    def find_next_point_in_line(self, path_to_goal):
        x1 = path_to_goal.poses[0].pose.position.x
        y1 = path_to_goal.poses[0].pose.position.y
        dx = path_to_goal.poses[-1].pose.position.x - path_to_goal.poses[0].pose.position.x
        dy = path_to_goal.poses[-1].pose.position.y - path_to_goal.poses[0].pose.position.y
        d2 = dx*dx + dy*dy
        nx = ((self.robot_pose.position.x-path_to_goal.poses[0].pose.position.x)*dx + (self.robot_pose.position.y-path_to_goal.poses[0].pose.position.y)*dy) / d2
        nx = min(1, max(0, nx))
        x3 = dx*nx + path_to_goal.poses[0].pose.position.x
        y3 = dy*nx + path_to_goal.poses[-1].pose.position.y
        if nx >0 :
            pathind= int(np.ceil(math.hypot(x3-x1, y3-y1)/self.granularity))
        else:
            pathind=0
        #print (x3, y3, nx, pathind)
        
        return pathind
        #print (progress)

        

    def follow_path(self, path_to_goal):
        
        print "Align Orientation first"
        self.align_orientation(path_to_goal)

        print "Find next point in line"
        start_goal = self.find_next_point_in_line(path_to_goal)

        print "Now go forwards!!"
        self.go_forwards(path_to_goal, start_goal)


    def _send_velocity_commands(self, xvel, yvel, angvel):
        cmd_vel = Twist()
        cmd_vel.linear.x = xvel
        cmd_vel.linear.y = yvel
        cmd_vel.angular.z= angvel
        self.cmd_pub.publish(cmd_vel)



    def _get_vector_to_pose(self, pose):
        transform = self._tf_listerner.transformPose('base_link',pose)
        
        orientation_list = [transform.pose.orientation.x, transform.pose.orientation.y, transform.pose.orientation.z, transform.pose.orientation.w]
        euls = tf.transformations.euler_from_quaternion(orientation_list)
        #print transform.pose.position.x, transform.pose.position.y, math.degrees(euls[2]), euls[2]
        self.ref_pub.publish(transform)
        
        ang_diff = euls[2]
        
        if self.backwards_mode:        
            if ang_diff>0:
                ang_diff=-(math.pi-ang_diff)
            else:
                ang_diff=-(-math.pi-ang_diff)
        
        
        return transform.pose.position.x, transform.pose.position.y, ang_diff


    def executeCallback(self, goal):
        self.backwards_mode=False
        self.cancelled = False
        initial_pose=self.get_node_position(self.closest_node)        
        final_pose=goal.target_pose.pose
        path_to_goal=self._get_path(initial_pose, final_pose)
        self.ppub.publish(path_to_goal)
        self.follow_path(path_to_goal)


        if not self.cancelled:
            self._as.set_succeeded()
        else:
            self._as.set_preempted(self._result)

    def preemptCallback(self):
        self.cancelled = True
        self._send_velocity_commands(0.0, 0.0, 0.0)
        self.backwards_mode=False
        #self._result.success = False
#        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node('row_traversal')
    server = inRowTravServer(rospy.get_name())