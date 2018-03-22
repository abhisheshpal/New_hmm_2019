#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import rospy
import geometry_msgs.msg
import rasberry_des.msg
import tf


class Robot(object):
    """Robot class definition"""
    def __init__(self, robot_id, transportation_rate, max_n_trays, env, farm, pickers, sim_rt_factor=1.0):
        self.robot_id = robot_id
        self.env = env
        self.farm = farm
        self.pickers = pickers
        self.transportation_rate = transportation_rate * sim_rt_factor
        self.n_empty_trays = max_n_trays
        self.n_full_trays = 0

        self.pose_pub = rospy.Publisher('/%s/pose' %(self.robot_id),
                                        geometry_msgs.msg.Pose,
                                        queue_size=10)
        self.pose = geometry_msgs.msg.Pose()

        self.status_pub = rospy.Publisher('/%s/status' %(self.robot_id),
                                        rasberry_des.msg.Robot_Status,
                                        queue_size=10)
        self.status = rasberry_des.msg.Robot_Status()
        self.status.robot_id = self.robot_id
        self.status.transportation_rate = transportation_rate
        self.status.n_empty_trays = self.n_empty_trays
        self.status.n_full_trays = self.n_full_trays

        self.prev_pub_time = 0.0
        self.pub_delay = 0.1

    def publish_pose(self, position, orientation):
        """This method publishes the current position of the picker. Called only at nodes"""
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]
        quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1],
                                                              orientation[2])
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.pose)

        # update variables in status and publish
        self.status.n_empty_trays = self.n_empty_trays
        self.status.n_full_trays = self.n_full_trays
        self.status.mode = self.mode
        self.status_pub.publish(self.status)

        self.prev_pub_time = self.env.now