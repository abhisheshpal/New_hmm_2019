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
    def __init__(self, robot_id, transportation_rate, max_n_trays, unloading_time, env,
                 topo_graph, des_env, sim_rt_factor=1.0):
        self.robot_id = robot_id
        self.env = env
        self.graph = topo_graph
        self.transportation_rate = transportation_rate * sim_rt_factor
        self.max_n_trays = max_n_trays
        self.n_empty_trays = self.max_n_trays
        self.n_full_trays = 0
        self.tot_trays = 0

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

        # service / client
        self.trays_loaded_service = rospy.Service("%s/tray_loaded" %(self.robot_id), rasberry_des.srv.Trays_Full, self.update_trays_loaded)
        rospy.loginfo("%s initialised service '%s/robot_info'" %(self.robot_id, self.robot_id))
        self.load_info = []
        self.trays_loaded_reponse = rasberry_des.srv.Trays_FullResponse()

        # action server / client
        self.collection_action = actionlib.SimpleActionServer("%s/collection" %(self.robot_id),
                                                              rasberry_des.msg.Robot_CollectionAction,
                                                              execute_cb=self.collect_n_unload, auto_start=False)
        rospy.loginfo("%s initialised action server '%s/collection'" %(self.robot_id, self.robot_id))
        self.collection_goal = rasberry_des.msg.Robot_CollectionGoal()
        self.collection_result = rasberry_des.msg.Robot_CollectionResult()
        self.collection_result.robot_id = self.robot_id
        self.collection_feedback = rasberry_des.msg.Robot_CollectionFeedback()
        self.collection_feedback.robot_id = self.robot_id
        self.collection_action.start()

        self.mode = 0   # 0 - free, 1 - busy, 2 - charging
        self.loaded = False

        # TODO: local storage node of the first row is assumed to be the starting loc
        # After reaching another local storage, the robot can wait there
        self.curr_node = self.graph.local_storage_nodes[self.graph.row_ids[0]]

        if des_env == "simpy":
            self.pub_delay = 1.0
            self.process_timeout = 0.001
            self.loop_timeout = 1.0
        elif des_env == "ros":
            self.pub_delay = max(0.25, 0.25 / sim_rt_factor)
            self.process_timeout = 0.001
            self.loop_timeout = 0.1

        self.action = self.env.process(self.normal_operation())

    def normal_operation(self, ):
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        curr_node_obj = self.graph.get_node(self.curr_node)
        position[0] = curr_node_obj.pose.position.x
        position[1] = curr_node_obj.pose.position.y

        while True:
            now_time = self.env.now
            if now_time - self.prev_pub_time >= self.pub_delay:
                self.publish_pose(position, orientation)
            yield self.env.timeout(self.loop_timeout)

    def update_trays_loaded(self, srv):
        self.load_info.append((srv.picker_id, srv.n_trays))
        self.n_empty_trays -= srv.n_trays
        self.n_full_trays += srv.n_trays
        self.loaded = True
        return self.tray_loaded_resposnse()

    def collect_n_unload(self, goal):
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]

        self.collection_goal = goal
        self.mode = 1

        # feedbacks
        _, _, distance_to_picker = self.graph.get_path_details(self.curr_node, self.collection_goal.picker_node)
        self.collection_feedback.eta_picker_node = sum(distance_to_picker) / self.transportation_rate
        _, _, distance_to_storage = self.graph.get_path_details(self.collection_goal.picker_node, self.collection_goal.local_storage_node)
        self.collection_feedback.eta_local_storage_node = sum(distance_to_storage) / self.transportation_rate
        self.collection_action.publish_feedback(self.collection_feedback)

        if (not self.collection_action.is_active()) or (self.collection_action.is_preempt_requested()):
            rospy.loginfo("%s's goal is inactive or pre-empted" %(self.robot_id))
            return

        # go to the picker_node
        self.go_to_node_with_feedback(self.collection_goal.picker_node, "picker")

        # picker knows which robot is coming and when the robot's pose is his node, he will
        # request the trays_unload service from farm, which in turn will call the service tray_loaded
        # this will set self.loaded
        while not self.loaded:
            curr_node_obj = self.graph.get_node(self.curr_node)
            position[0] = curr_node_obj.pose.position.x
            position[1] = curr_node_obj.pose.position.y
            if self.prev_pub_time - self.env.now >= self.pub_delay:
                self.publish_pose(position, orientation)
                self.collection_action.publish_feedback(self.collection_feedback)
            pass

        # publish pose and feedback
        self.publish_pose(position, orientation)
        self.collection_action.publish_feedback(self.collection_feedback)

        # go to local storage node
        # wait for unloading time
        # send success
        self.go_to_node_with_feedback(self.collection_goal.local_storage_node, "storage")
        wait_time = self.unloading_time * self.n_full_trays
        self.wait_with_feedback(wait_time)

        # update tray counts
        self.tot_trays += self.n_full_trays
        self.n_empty_trays += self.n_full_trays
        self.n_full_trays = 0

        self.loaded = False
        self.mode = 0

        # send success
#        self.collection_result.success = True
        self.collection_action.set_succeeded(self.collection_result)

    def wait_with_feedback(self, wait_time):
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        curr_node_obj = self.graph.get_node(self.curr_node)
        position[0] = curr_node_obj.pose.position.x
        position[1] = curr_node_obj.pose.position.y

        self.collection_feedback.eta_picker_node = 0.
        self.collection_feedback.eta_storage_node = 0.

        start_time = self.env.now
        delta_time = self.env.now - start_time

        while delta_time <= wait_time:
            now_time = self.env.now
            if now_time - self.prev_pub_time >= self.pub_delay:
                self.publish_pose(position, orientation)
                self.collection_action.publish_feedback(self.collection_feedback)
            delta_time = now_time - start_time
            yield self.env.timeout(self.loop_timeout)

    def go_to_node_with_feedback(self, goal_node, stage):
        """Simpy process to Mimic moving to the goal_node by publishing new position

        Keyword arguments:
        goal_node -- node to reach from current node
        """
        route_nodes, route_edges, route_distance = self.graph.get_path_details(self.curr_node,
                                                                                    goal_node)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        for i in range(len(route_nodes) - 1):
            # move through each edge
            curr_node_obj = self.graph.get_node(route_nodes[i])
            next_node_obj = self.graph.get_node(route_nodes[i + 1])
            theta = math.atan2((next_node_obj.pose.position.y - curr_node_obj.pose.position.y),
                               (next_node_obj.pose.position.x - curr_node_obj.pose.position.x))

            position[0] = curr_node_obj.pose.position.x
            position[1] = curr_node_obj.pose.position.y

            edge_distance = route_distance[i]
            travel_time = edge_distance / self.transportation_rate

            eta = sum(route_distance[i + 1:]) / self.transportation_rate

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

                print position[0], position[1]

                now_time = self.env.now
                if now_time - self.prev_pub_time >= self.pub_delay:
                    self.publish_pose(position, orientation)
                    if stage == "picker":
                        self.collection_feedback.eta_picker_node = eta + (travel_time - delta_time)
                    elif stage == "storage":
                        self.collection_feedback.eta_storage_node = eta + (travel_time - delta_time)
                    self.collection_action.publish_feedback(self.collection_feedback)
                delta_time = now_time - start_time
                rospy.sleep(self.loop_timeout)
            self.curr_node = route_nodes[i + 1]

        self.publish_pose(position, orientation)
        if stage == "picker":
            self.collection_feedback.eta_picker_node = 0.
        elif stage == "storage":
            self.collection_feedback.eta_storage_node = 0.
        self.collection_action.publish_feedback(self.collection_feedback)

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

        # status publisher
        self.status.n_empty_trays = self.n_empty_trays
        self.status.n_full_trays = self.n_full_trays
        self.status.tot_trays = self.tot_trays
        self.status.mode = self.mode
        self.status.loaded = self.loaded
        self.status_pub.publish(self.status)

        self.prev_pub_time = self.env.now
