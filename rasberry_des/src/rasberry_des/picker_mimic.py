#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import rospy
import rasberry_des.picker
import std_msgs.msg
import geometry_msgs.msg



class PickerMimic(rasberry_des.picker.Picker):
    """PickerMimic class to mimic long picking operations
    """
    def __init__(self, picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, verbose=False):
        """Initialise PickerMimic class
        """
        super(PickerMimic, self).__init__(picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, verbose)
        self.closest_node_pub = rospy.Publisher("%s/closest_node" %(self.picker_id), std_msgs.msg.String, latch=True, queue_size=10)
        self.current_node_pub = rospy.Publisher("%s/current_node" %(self.picker_id), std_msgs.msg.String, latch=True, queue_size=10)
        self.pose_pub = rospy.Publisher("%s/pose" %(self.picker_id), geometry_msgs.msg.PoseStamped, latch=True, queue_size=10)
        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "map"

        self.publish_pose(self.curr_node)


    def go_to_node(self, goal_node, nav_speed):
        """Simpy process to Mimic moving to the goal_node by publishing new position based on speed

        Keyword arguments:
        goal_node -- node to reach from current node
        nav_speed -- navigation speed (picking / transportation rate)
        """
        self.loginfo("%s going to %s from %s" %(self.picker_id, goal_node, self.curr_node))
        route_nodes, _, route_distance = self.graph.get_path_details(self.curr_node, goal_node)
        for i in range(len(route_nodes) - 1):
            # move through each edge
            edge_distance = route_distance[i]
            travel_time = edge_distance / nav_speed
            yield self.env.timeout(travel_time)

            self.curr_node = route_nodes[i + 1]
            self.publish_pose(self.curr_node)

        self.loginfo("%s reached %s" %(self.picker_id, goal_node))

        yield self.env.timeout(self.process_timeout)

    def publish_pose(self, node):
        """
        """
        self.closest_node_pub.publish(node)
        self.current_node_pub.publish(node)
        self.pose.pose.position = self.graph.get_node(node).pose.position
        if self.picking_dir is None or self.picking_dir == "forward":
            self.pose.pose.orientation.x = 0.
            self.pose.pose.orientation.y = 0.
            self.pose.pose.orientation.z = 0.
            self.pose.pose.orientation.w = 1.
        else:
            self.pose.pose.orientation.x = 0.
            self.pose.pose.orientation.y = 0.
            self.pose.pose.orientation.z = 1.
            self.pose.pose.orientation.w = 0.
        self.pose_pub.publish(self.pose)

        rospy.sleep(0.1)
