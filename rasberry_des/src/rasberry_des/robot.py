#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import math
import rospy
import geometry_msgs.msg
import rasberry_des.msg
import tf
import actionlib


class Robot(object):
    """Robot class definition"""
    def __init__(self, robot_id, transportation_rate, max_n_trays, unloading_time, env, farm, des_env, sim_rt_factor=1.0):
        self.robot_id = robot_id
        self.env = env
        self.farm = farm
        self.transportation_rate = transportation_rate * sim_rt_factor
        self.max_n_trays = max_n_trays
        self.n_empty_trays = self.max_n_trays
        self.n_full_trays = 0

        # publishers / subscribers
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
        self.unloading_time = unloading_time
        self.status.n_empty_trays = self.n_empty_trays
        self.status.n_full_trays = self.n_full_trays

        self.prev_pub_time = 0.0

        # action server / client
        self.collection_action = actionlib.SimpleActionServer("%s/collection" %(self.robot_id),
                                                              rasberry_des.msg.Robot_CollectionAction,
                                                              execute_cb=self.collect_n_unload, auto_start=False)
        self.collection_goal = rasberry_des.msg.Robot_CollectionGoal()
        self.collection_result = rasberry_des.msg.Robot_CollectionResult()
        self.collection_feedback = rasberry_des.msg.Robot_CollectionFeedback()
        self.collection_action.start()

        self.mode = 0   # 0 - free, 1 - busy, 2 - charging

        # TODO: local storage node of the first row is assumed to be the starting loc
        # After reaching another local storage, the robot can wait there
        self.curr_node = self.farm.graph.local_storage_nodes[self.farm.row_ids[0]]

        if des_env == "simpy":
            self.pub_delay = 1.0
            self.process_timeout = 0.001
            self.loop_timeout = 1.0
        elif des_env == "ros":
            self.pub_delay = max(0.1, 0.1 / sim_rt_factor)
            self.process_timeout = 0.001
            self.loop_timeout = 0.05

    def collect_n_unload(self, goal):
        self.collection_goal = goal
        self.mode = 1

        # feedbacks
        _, _, distance_to_picker = self.farm.graph.get_path_details(self.curr_node, self.collection_goal.picker_node)
        self.collection_feedback.eta_picker_node = sum(distance_to_picker) / self.transportation_rate
        _, _, distance_to_storage = self.farm.graph.get_path_details(self.collection_goal.picker_node, self.collection_goal.local_storage_node)
        self.collection_feedback.eta_local_storage_node = sum(distance_to_storage) / self.transportation_rate
        self.collection_feedback.eta_local_storage_node += self.collection_goal.n_trays * self.unloading_time
        self.collection_action.publish_feedback(self.collection_feedback)

        # go to the picker_node
        yield self.env.process(self.go_to_node(self.collection_goal.picker_node))

        # TODO: This will be replaced with a service call to the picker
        curr_node_obj = self.farm.graph.get_node(self.curr_node)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        position[0] = curr_node_obj.pose.position.x
        position[1] = curr_node_obj.pose.position.y

        start_time = self.env.now
        wait_time = 10
        delta_time = self.env.now - start_time
        while delta_time <= wait_time:
            if self.prev_pub_time - self.env.now >= self.pub_delay:
                self.publish_pose(position, orientation)
                # TODO: publish feedback
            yield self.env.timeout(self.loop_timeout)

        # after the loading time, the tray counts are modified
        self.n_empty_trays -= self.collection_goal.n_trays
        self.n_full_trays += self.collection_goal.n_trays
        self.publish_pose(position, orientation)
        # TODO: publish feedback

        # go to local storage node and send success

    def go_to_node_with_feedback(self, goal_node):
        """Simpy process to Mimic moving to the goal_node by publishing new position

        Keyword arguments:
        goal_node -- node to reach from current node
        """
        #TODO: update to send feedback
        route_nodes, route_edges, route_distance = self.farm.graph.get_path_details(self.curr_node,
                                                                                    goal_node)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        for i in range(len(route_nodes) - 1):
            # move through each edge
            curr_node_obj = self.farm.graph.get_node(route_nodes[i])
            next_node_obj = self.farm.graph.get_node(route_nodes[i + 1])
            theta = math.atan2((next_node_obj.pose.position.y - curr_node_obj.pose.position.y),
                               (next_node_obj.pose.position.x - curr_node_obj.pose.position.x))

            position[0] = curr_node_obj.pose.position.x
            position[1] = curr_node_obj.pose.position.y

            edge_distance = route_distance[i]
            travel_time = edge_distance / self.transportation_rate

            self.publish_pose(position, orientation)
            start_time = self.env.now
            delta_time = self.env.now - start_time

            while delta_time <= travel_time:
                delta = self.transportation_rate * delta_time
                if delta <= edge_distance:
                    position[0] = curr_node_obj.pose.position.x + delta * math.cos(theta)
                    position[1] = curr_node_obj.pose.position.y + delta * math.sin(theta)
                else:
                    position[0] = next_node_obj.pose.position.x
                    position[1] = next_node_obj.pose.position.y

                now_time = self.env.now
                if now_time - self.prev_pub_time >= self.pub_delay:
                    self.publish_pose(position, orientation)
                delta_time = now_time - start_time
                yield self.env.timeout(self.loop_timeout)
            self.curr_node = route_nodes[i + 1]

        yield self.env.timeout(self.process_timeout)

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

        # TODO: update variables in status and publish
        self.status.n_empty_trays = self.n_empty_trays
        self.status.n_full_trays = self.n_full_trays
        self.status.mode = self.mode
        self.status_pub.publish(self.status)

        self.prev_pub_time = self.env.now
