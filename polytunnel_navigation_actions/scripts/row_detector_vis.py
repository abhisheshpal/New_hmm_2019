#!/usr/bin/env python
import rospy, tf
from polytunnel_navigation_actions.msg import ObstacleArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

class row_detector_vis(object):
    
    
    def __init__(self):
        
        rospy.Subscriber("/row_detector/poles", ObstacleArray, self.poles_callback,  queue_size=1)
        rospy.Subscriber("/row_detector/obstacles", ObstacleArray, self.obstacles_callback,  queue_size=1)
        self.listener = tf.TransformListener()
        self.poles_vis_pub = rospy.Publisher('/row_detector/poles_vis', MarkerArray, queue_size=1)
        self.obs_vis_pub = rospy.Publisher('/row_detector/obstacles_vis', MarkerArray, queue_size=1)
        

    def obstacles_callback(self, msg):
        obstacle_id = 0
        pole_vis_array = []
        for obs in msg.obstacles:
            map_pose = self._transform_to_pose_stamped(obs)
            
            pole_vis = Marker()
            pole_vis = self.fill_marker_msg(map_pose, pole_vis, obstacle_id, radius=obs.radius, colour=[0.7, 0.2, 0.2])
            pole_vis_array.append(pole_vis)
            
            obstacle_id+=1
            
        poles_vis = MarkerArray()
        poles_vis = pole_vis_array
        
        #r = rospy.Rate(3)
        #while not rospy.is_shutdown():
        self.poles_vis_pub.publish(poles_vis)  
        #r.sleep()    
        


    def poles_callback(self, msg):
#        print "b"
#        now = rospy.Time.now()
        #self.listener.waitForTransform("/base_link", "/map", now, rospy.Duration(4.0))
        #(trans,rot) = self.listener.lookupTransform("/base_link", "/map", now)
        
        obstacle_id = 0
        pole_vis_array = []
        for pole in msg.obstacles:
            map_pose = self._transform_to_pose_stamped(pole)
            
            pole_vis = Marker()
            pole_vis = self.fill_marker_msg(map_pose, pole_vis, obstacle_id)
            pole_vis_array.append(pole_vis)
            
            obstacle_id+=1
            
        poles_vis = MarkerArray()
        poles_vis = pole_vis_array
        
        #r = rospy.Rate(3)
        #while not rospy.is_shutdown():
        self.poles_vis_pub.publish(poles_vis)  
        #r.sleep()    
    
    
    def _transform_to_pose_stamped(self, pose_2d):
        the_pose = PoseStamped()
        the_pose.header.frame_id = 'base_link'
        the_pose.pose.position.x = pose_2d.pose.x#-1.0*pose_2d.pose.x
        the_pose.pose.position.y = pose_2d.pose.y
        the_pose.pose.position.z = 0.5
        the_pose.pose.orientation.w = 1.0#tf.transformations.quaternion_from_euler(0.0,0.0,pose_2d.pose.theta)
        map_pose = self.listener.transformPose('map', the_pose)
        return map_pose


    def fill_marker_msg(self, map_pose, msg, id_, radius=0.07, colour=[0.5, 0.5, 0.5]):
        
        msg.type = 3
        msg.header.frame_id = map_pose.header.frame_id
        msg.id = id_
        msg.pose.position.x = map_pose.pose.position.x
        msg.pose.position.y = map_pose.pose.position.y
        msg.pose.position.z = map_pose.pose.position.z
        msg.pose.orientation = map_pose.pose.orientation
        msg.scale.x = radius
        msg.scale.y = radius
        msg.scale.z = 1.0
        msg.color.a = 0.9
        msg.color.r = colour[0]
        msg.color.g = colour[1]
        msg.color.b = colour[2]
        msg.lifetime = rospy.Duration(0.3)
        #msg.frame_locked = True
        
        return msg
###################################################################################


#####################################################################################
if __name__ == "__main__":
    
    rospy.init_node("row_detector_vis", anonymous=True)
    
    rdv = row_detector_vis()
    
    rospy.spin()
#####################################################################################