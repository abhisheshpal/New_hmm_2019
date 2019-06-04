#!/usr/bin/env python

import rospy
import math
import PyKDL

import numpy as np
import matplotlib.path as mplPath

import tf

import actionlib
import polytunnel_navigation_actions.msg
import std_msgs.msg


from std_srvs.srv import SetBool

from dynamic_reconfigure.server import Server
from polytunnel_navigation_actions.cfg import RowTraversalConfig

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from strands_navigation_msgs.msg import TopologicalMap
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import Marker
#from visualization_msgs.msg import MarkerArray

class inRowTravServer(object):

    _feedback = polytunnel_navigation_actions.msg.inrownavFeedback()
    _result   = polytunnel_navigation_actions.msg.inrownavResult()

    def __init__(self, name):
        self.colision=False
        self.giveup_timer_active=False
        self.notification_timer_active=False
        self.notified=False

        self.kp_ang_ro= 0.6                     # Proportional gain for initial orientation target
        self.constant_forward_speed = True      # Stop when obstacle in safety area (no slowdown) **WIP**
        self.initial_heading_tolerance = 0.005  # Initial heading tolerance [rads]
        self.kp_ang= 0.2                        # Proportional gain for heading correction
        self.kp_y= 0.1                          # Proportional gain for sideways corrections
        self.granularity= 0.5                   # Distance between minigoals along path (carrot points)
        self.y_row_detection_bias = 0.7         # Weight given to the reference given by row detection
        self.y_path_following_bias = 0.3        # Weight given to the original path following
        self.ang_row_detection_bias = 0.2       # Weight given to the angular reference given by row detection
        self.ang_path_following_bias = 0.8      # Weight given to the angular refernce given by path following
        self.minimum_turning_speed = 0.01       # Minimum turning speed
        self.emergency_clearance_x = 0.22       # Clearance from corner frames to trigger emergency stop in x
        self.emergency_clearance_y = 0.22       # Clearance from corner frames to trigger emergency stop in y
        self.forward_speed= 0.8                 
        self.quit_on_timeout=False
        self.time_to_quit=10.0                  # Time until the action is cancelled since collision detected
        
        # This dictionary defines which function should be called when a variable changes via dynamic reconfigure
        self._reconf_functions={'variables':['emergency_clearance_x', 'emergency_clearance_y'], 
                                'functions':[self.define_safety_zone, self.define_safety_zone]}
        
        self.laser_emergency_regions=[]
        self.redefine_laser_regions=False
        self.limits=[]
        self.emergency_base_points=[]           # Corners of Emergency Areas
        self.y_ref=None
        self.ang_ref=None
        self.config={}
        self.cancelled = False
        self._action_name = name
        self._got_top_map=False
        self.lnodes=None
        self.backwards_mode=False
        self.safety_marker=None
        self.active=True

        while not self.lnodes and not self.cancelled:
            rospy.loginfo("Waiting for topological map")
            rospy.Subscriber('/topological_map', TopologicalMap, self.topological_map_cb)
            if not self.lnodes:
                rospy.sleep(1.0)

        rospy.Subscriber('/scan', LaserScan, self.laser_cb, queue_size=1)
        rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
        rospy.Subscriber('/closest_node', std_msgs.msg.String, self.closest_node_cb)
        rospy.Subscriber('/row_detector/path_error',Pose2D, self.row_correction_cb)

        self._tf_listerner = tf.TransformListener()
        self._activate_srv = rospy.ServiceProxy('/row_detector/activate_detection', SetBool)
        
        self.ppub = rospy.Publisher('/row_traversal/row_line', Path, queue_size=1)
        self.cmd_pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.ref_pub = rospy.Publisher('/row_traversal/goal_reference', PoseStamped, queue_size=1)
        self.safety_zone_vis_pub = rospy.Publisher('/row_traversal/safety_zone', Marker, queue_size=1)
        self.not_pub = rospy.Publisher('/row_traversal/notification', String, queue_size=1)

        self.dyn_reconf_srv = Server(RowTraversalConfig, self.dyn_reconf_callback)


        rospy.loginfo("Creating safety zone.")
        self.define_safety_zone()


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
                if lk[0] in self._reconf_functions['variables']:
                    self._reconf_functions['functions'][self._reconf_functions['variables'].index(lk[0])]()

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



    def define_safety_zone(self, corner_frames=['top0','top1','top2','top3']):
        """ Defines the Safety zone around the robot 
        
        Arguments:
        clearance     -- The outwards distance trom the corner_frames at which the vertices of the safety zone are defined
        corner_frames -- The name of the frames that define the extremes of the robot
        """
        print "Defining Safety Zone"
        self.limits=[]
        self.emergency_base_points=[]
        
        for i in corner_frames:
            d={}
            self._tf_listerner.waitForTransform('base_link',i,rospy.Time.now(), rospy.Duration(1.0))
            (trans,rot) = self._tf_listerner.lookupTransform('base_link',i,rospy.Time.now())
            i_x, i_y, i_z = trans
            cpi=PointStamped()
            cpi.header.frame_id='base_link'
            cpi.point.x= i_x+self.emergency_clearance_x if i_x > 0 else  i_x-self.emergency_clearance_x
            cpi.point.y= i_y+self.emergency_clearance_y if i_y > 0 else  i_y-self.emergency_clearance_y
            cpi.point.z=-0.3
            d['point']=cpi
            d['angle']=math.atan2(cpi.point.y,cpi.point.x)
            self.emergency_base_points.append(d)
        
        # We sort in angle from move base
        self.emergency_base_points = sorted( self.emergency_base_points, key=lambda k: k['angle']) 
        self.redefine_laser_regions = True
        print "visualise safety zone"
        self.safety_zones_visualisation()
    
    
    def laser_cb(self, msg):
        if self.redefine_laser_regions:
            self.safety_zones_find_laser_regions(msg)
        elif self.laser_emergency_regions and self.active :
            min_range = min(x for x in msg.ranges if x > msg.range_min) # Necessary in case there are -1 in data
            self.colision=False
            #print "min range: ", min_range, " -> ", self.max_emergency_dist#, " of ", len(minslist)
            if min_range<=self.max_emergency_dist:
                minslist = [(x, msg.ranges.index(x)) for x in msg.ranges if x <= self.max_emergency_dist]
                #print "min range: ", min_range, " -> ", self.max_emergency_dist, " of ", len(minslist)
                for i in minslist:
                    angle = (i[1]*msg.angle_increment)-msg.angle_min
                    p1=[(i[0]*np.cos(angle)),(i[0]*np.sin(angle))]
                    path = mplPath.Path(self.emergency_poly)
                    inside2 = path.contains_points([p1])
                    if inside2:
                        self.colision=True
                        self._send_velocity_commands(0.0, 0.0, 0.0)
                        degang = np.rad2deg(angle)
                        if degang>=360.0:
                            degang=degang-360.0
                        #colstr = "HELP!: Colision "+ str(degang) +" "+ str(i[0])+" "+ str(rospy.Time.now().secs)
                        #print colstr
                        if self.quit_on_timeout and not self.giveup_timer_active:
                            self.timer = rospy.Timer(rospy.Duration(self.time_to_quit), self.giveup, oneshot=True)
                            self.giveup_timer_active=True
                        
                        if not self.notified and not self.notification_timer_active:
                            self.timer = rospy.Timer(rospy.Duration(3.0), self.nottim, oneshot=True)
                            self.notification_timer_active=True
                        #self.not_pub.publish(colstr)
                        break
            if not self.colision:
                self.notified=False


    def nottim(self, timer):
        colstr = "HELP!: Colision near "+ str(self.closest_node) +" at "+ str(rospy.Time.now().secs)
        self.not_pub.publish(colstr)
        self.notification_timer_active=False
        self.notified=True
        
    
    def safety_zones_find_laser_regions(self, msg):
        self.laser_emergency_regions=[]
        for i in range(len(self.emergency_base_points)-1):
            d = {}
            d['range'] = []
            d['range'].append(int(np.floor((self.emergency_base_points[i]['angle']-msg.angle_min)/msg.angle_increment)))
            d['range'].append(int(np.floor((self.emergency_base_points[i+1]['angle']-msg.angle_min)/msg.angle_increment)))
            midx=(self.emergency_base_points[i]['point'].point.x + self.emergency_base_points[i+1]['point'].point.x)/2.0
            midy=(self.emergency_base_points[i]['point'].point.y + self.emergency_base_points[i+1]['point'].point.y)/2.0
            d['dist']= math.hypot(self.emergency_base_points[i]['point'].point.x, self.emergency_base_points[i+1]['point'].point.y)
            d['mean_dist']=math.hypot(midx, midy)
            self.laser_emergency_regions.append(d)

        d = {}
        d['range'] = []
        d['range'].append(int(np.floor((self.emergency_base_points[-1]['angle']-msg.angle_min)/msg.angle_increment)))
        d['range'].append(int(np.floor((self.emergency_base_points[0]['angle']-msg.angle_min)/msg.angle_increment)))
        midx=(self.emergency_base_points[0]['point'].point.x + self.emergency_base_points[-1]['point'].point.x)/2.0
        midy=(self.emergency_base_points[0]['point'].point.y + self.emergency_base_points[-1]['point'].point.y)/2.0
        d['dist']= d['dist']= math.hypot(self.emergency_base_points[0]['point'].point.x, self.emergency_base_points[-1]['point'].point.y)
        d['mean_dist']=math.hypot(midx, midy)
        self.laser_emergency_regions.append(d)

        self.emergency_poly=[]        
        for i in self.laser_emergency_regions:
            print i
        
        for i in self.emergency_base_points:
            r=(i['point'].point.x, i['point'].point.y)
            self.emergency_poly.append(r)
            
        self.emergency_poly=np.asarray(self.emergency_poly)
        self.redefine_laser_regions=False
        
        self.max_emergency_dist= 0.0
        for i in self.laser_emergency_regions:
            print i['dist']
            self.max_emergency_dist=np.max([self.max_emergency_dist, i['dist']])
#        laser_angles.append(msg.angle_max)
            
    
    def safety_zones_visualisation(self):

        base_pose = Pose()
        base_pose.orientation.w=1.0

        amarker = Marker()
        amarker.header.frame_id = "base_link"
        #amarker.header.stamp = rospy.Time.now()
        amarker.type = 4
        amarker.pose = Pose()
        amarker.pose.position.z = 0.51
        amarker.scale.x = 0.05
        amarker.color.a = 0.5
        amarker.color.r = 0.9
        amarker.color.g = 0.1
        amarker.color.b = 0.1
        amarker.lifetime = rospy.Duration(0.0)
        amarker.frame_locked = True
        
        for i in self.emergency_base_points:
            amarker.points.append(i['point'].point)
        amarker.points.append(self.emergency_base_points[0]['point'].point)
               
        self.safety_marker=amarker


    def row_correction_cb(self, msg):
        if np.isnan((msg.y)):
            self.y_ref=None
        else:
            self.y_ref=msg.y

        if np.isnan((msg.theta)):
            self.ang_ref=None
        else:
            self.ang_ref=msg.theta



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

        dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[0])
        if np.abs(ang_diff) >= self.initial_heading_tolerance:
            print "INITIAL ANg DIFF: ", np.abs(ang_diff)
            while np.abs(ang_diff) >= self.initial_heading_tolerance and not self.cancelled:
                self._send_velocity_commands(0.0, 0.0, self.kp_ang_ro*ang_diff, consider_minimum_rot_vel=True)
                rospy.sleep(0.05)
                dist, y_err, ang_diff = self._get_vector_to_pose(path_to_goal.poses[0])

            self._send_velocity_commands(0.0, 0.0, 0.0)
            rospy.sleep(2.0) # We need to give time to the wheels to straighten

        print "Done: ", ang_diff
        

    def go_forwards(self, path_to_goal, start_goal):        
        if self.backwards_mode:
            speed = -self.forward_speed
        else:
            speed = self.forward_speed
        print "Number of intermediate goals: ",start_goal, len(path_to_goal.poses)
        for i in range(start_goal, len(path_to_goal.poses)):
            dist, y_err, ang_diff = self._get_references(path_to_goal.poses[i])         
            while np.abs(dist)>0.1 and not self.cancelled:
                self._send_velocity_commands(speed, self.kp_y*y_err, self.kp_ang*ang_diff)
                rospy.sleep(0.05)
                #self._get_vector_to_pose(path_to_goal.poses[i])
                dist, y_err, ang_diff = self._get_references(path_to_goal.poses[i])
                
            if not self.cancelled:
                print("Next Goal")
            else:
                break

        if not self.cancelled:
            self._send_velocity_commands(0.0, 0.0, 0.0)
            self.active=False

        
    def find_next_point_in_line(self, path_to_goal):
        '''
        Closest point to line method
        '''
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
            pathind= int(np.floor(math.hypot(x3-x1, y3-y1)/self.granularity))
        else:
            pathind=0
        #print (x3, y3, nx, pathind)
        
        return pathind


    def follow_path(self, path_to_goal):
        
        print "Align Orientation first"
        self.align_orientation(path_to_goal)

        print "Find next point in line"
        start_goal = self.find_next_point_in_line(path_to_goal)

        print "Now go forwards!!"
        self.go_forwards(path_to_goal, start_goal)


    def _send_velocity_commands(self, xvel, yvel, angvel, consider_minimum_rot_vel=False):
        #print self.colision
        if not self.colision:
            cmd_vel = Twist()
            cmd_vel.linear.x = xvel
            cmd_vel.linear.y = yvel
            if consider_minimum_rot_vel:
                if np.isclose(angvel, 0.0):
                    cmd_vel.angular.z = 0.0            
                elif np.abs(angvel) >= self.minimum_turning_speed:
                    cmd_vel.angular.z = angvel
                else:
                    if angvel > 0.0:
                        cmd_vel.angular.z = self.minimum_turning_speed 
                    elif angvel < 0.0:
                        cmd_vel.angular.z = -1.0 * self.minimum_turning_speed
    
            else:
                cmd_vel.angular.z = angvel
    
    #        print 'ANg: ', cmd_vel.angular.z
            self.cmd_pub.publish(cmd_vel)
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0            
            self.cmd_pub.publish(cmd_vel)



            

    def _get_references(self,pose):
        dist, y_path_err, ang_path_diff = self._get_vector_to_pose(pose)
        
        if self.y_ref:
#            if self.backwards_mode:
#                self.y_ref = -1.0*self.y_ref
            y_err = np.average([self.y_ref, y_path_err], weights=[self.y_row_detection_bias, self.y_path_following_bias])
        else:
            y_err = y_path_err


        if self.ang_ref:
            ang_diff = np.average([self.ang_ref, ang_path_diff], weights=[self.ang_row_detection_bias, self.ang_path_following_bias])
        else:
            ang_diff = ang_path_diff


        #print dist, y_err, ang_diff
        return dist, y_err, ang_diff


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

    def activate_row_detector(self, onoff):
        rospy.wait_for_service('/row_detector/activate_detection')
        try:
            activate_service = rospy.ServiceProxy('/row_detector/activate_detection', SetBool)
            resp1 = activate_service(onoff)
            print resp1.message
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e       


    def executeCallback(self, goal):
        rospy.loginfo("New goal received")
        self.backwards_mode=False
        self.cancelled = False
        self.active=True
        
        self.activate_row_detector(True)
        initial_pose=self.get_node_position(self.closest_node)        
        final_pose=goal.target_pose.pose
        path_to_goal=self._get_path(initial_pose, final_pose)
        self.ppub.publish(path_to_goal)
        self.follow_path(path_to_goal)

        self.activate_row_detector(False)
        if not self.cancelled:
            self._as.set_succeeded()
        else:
            self._as.set_preempted(self._result)


    def giveup(self, timer):
        if self.colision:
            rospy.loginfo("Row Traversal Cancelled")
            self.not_pub.publish("Row Traversal timedout after colision")
            self.cancelled = True
            self.backwards_mode=False
        self.giveup_timer_active=False
        
        
    def do_stop(self, timer):
        if not self.active:
            print "do_stop"
            self._send_velocity_commands(0.0, 0.0, 0.0)
            
        
        
    def preemptCallback(self):
        rospy.loginfo("Row Traversal Cancelled")
        self.cancelled = True
        self.timer = rospy.Timer(rospy.Duration(0.1), self.do_stop, oneshot=True)
        #self._send_velocity_commands(0.0, 0.0, 0.0)
        self.backwards_mode=False
        #self._result.success = False
#        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node('row_traversal')
    server = inRowTravServer(rospy.get_name())