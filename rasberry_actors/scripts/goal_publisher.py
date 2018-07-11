#!/usr/bin/env python
"""
Created on Wed Jun 20 13:29:21 2018

@author: adam
"""
#####################################################################################
import rospy
import move_base_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf
import actionlib


def get_transform(q, frame1, frame2):
    print "\n"
    print "Looking up transform between " + frame1 + " and " + frame2      
    for i in range(5):
        try:
            trans, rot = q.lookupTransform(frame1, frame2, rospy.Time(0))
        except:
            print i, "transform does not exist"
        else:
            print "OK"
            print trans
            print rot
     
        rospy.sleep(1) 
#####################################################################################        


#####################################################################################
# Send move_base_simple/goal topics - working :)

x = -12
y = 0
R = 0
P = 0
Y = -1.57

print_transform = False
#####################################################################################


#####################################################################################
rospy.init_node("goal_publisher")   
simple_goal = geometry_msgs.msg.PoseStamped()
pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=10)
 
for i in range(3):
    simple_goal.header.frame_id = "world"
    simple_goal.pose.position.x = x
    simple_goal.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(R,P,Y)
    simple_goal.pose.orientation.x = quat[0]
    simple_goal.pose.orientation.y = quat[1]
    simple_goal.pose.orientation.z = quat[2]
    simple_goal.pose.orientation.w = quat[3]
     
    pub.publish(simple_goal)
    rospy.sleep(1)
#####################################################################################


##################################################################################### 
if print_transform:
    rospy.sleep(2)
    q = tf.TransformListener()
    get_transform(q, "/map", "/actor00/hokuyo")
    get_transform(q, "/actor00/odom", "/actor00/hokuyo")
    get_transform(q, "/world", "/actor00/hokuyo")
####################################################################################



#     # move_base action goals - not working :(
#     
#     goal = move_base_msgs.msg.MoveBaseGoal()
#     
#     goal.target_pose.pose.position.x = 0.0
#     goal.target_pose.pose.position.y = 0.0
#     quat = tf.transformations.quaternion_from_euler(0,0,0.0)
#     goal.target_pose.pose.orientation.x = quat[0]
#     goal.target_pose.pose.orientation.y = quat[1]
#     goal.target_pose.pose.orientation.z = quat[2]
#     goal.target_pose.pose.orientation.w = quat[3]
#     
#     mb_client = actionlib.SimpleActionClient('/actor00/move_base', move_base_msgs.msg.MoveBaseAction)
#     mb_client.wait_for_server()
#     print "00"
#     
#     mb_client.send_goal(goal)
#     
#     mb_client.wait_for_result()
# 
#     print mb_client.get_result()    


## Checking transform exist from laser to map - working :)
#class Laser(object):
#    def __init__(self):
#        self.data = sensor_msgs.msg.LaserScan()
#    def update(self, msg):
#         """this method can be used to update the lase scan 
#         with in this object using the callback functionality
#         
#         Keyword arguments:
#         
#         msg - sensor_msgs.msg.LaserScan
#         """
#         self.data = msg
 
 
#laser = Laser()
#sub = rospy.Subscriber("/actor00/scan", sensor_msgs.msg.LaserScan, callback=laser.update)