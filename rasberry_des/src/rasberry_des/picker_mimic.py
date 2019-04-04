#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import random
import rospy
import rasberry_des.picker
import std_msgs.msg
import geometry_msgs.msg
import strands_executive_msgs.srv
import strands_executive_msgs.msg




class PickerMimic(rasberry_des.picker.Picker):
    """PickerMimic class to mimic long picking operations
    """
    def __init__(self, picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, with_robots, verbose=False):
        """Initialise PickerMimic class
        """
        super(PickerMimic, self).__init__(picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, verbose)

        self.with_robots = with_robots

        self.closest_node_pub = rospy.Publisher("%s/closest_node" %(self.picker_id), std_msgs.msg.String, latch=True, queue_size=10)
        self.current_node_pub = rospy.Publisher("%s/current_node" %(self.picker_id), std_msgs.msg.String, latch=True, queue_size=10)
        self.pose_pub = rospy.Publisher("%s/posestamped" %(self.picker_id), geometry_msgs.msg.PoseStamped, latch=True, queue_size=10)
        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "map"
        self.publish_pose(self.curr_node)

        rospy.wait_for_service("/rasberry_coordination/add_task")
        self.add_task_client = rospy.ServiceProxy("/rasberry_coordination/add_task", strands_executive_msgs.srv.AddTask)

        rospy.wait_for_service("/rasberry_coordination/cancel_task")
        self.cancel_task_client = rospy.ServiceProxy("/rasberry_coordination/cancel_task", strands_executive_msgs.srv.CancelTask)

        self.state = "INIT"

        self.car_event_sub = rospy.Subscriber("/car_client/get_states", std_msgs.msg.String, self.car_event_cb)

        self.car_state_pub = rospy.Publisher("/car_client/set_states", std_msgs.msg.String, latch=True, queue_size=5)

        self.set_picker_state(self.state)

    def car_event_cb(self, msg):
        """callback function for /car_client/get_states
        """
        msg_data = eval(msg.data)
        if "states" in msg_data:
            for picker_id in msg_data["states"]:
                if picker_id == self.picker_id:
                    # update state only if the state for this user has been changed
                    if self.state != msg_data["states"][picker_id]:
                        self.state = msg_data["states"][picker_id]

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

    def set_picker_state(self, state):
        msg = std_msgs.msg.String()
        msg.data = '{\"user\":\"%s\", \"state\": \"%s\"}' %(self.picker_id, state)
        self.car_state_pub.publish(msg)

    def normal_operation(self, ):
        """ Picker's picking process when there are robots to carry full trays
        """
        idle_start_time = self.env.now
        transportation_start_time = 0.
        picking_start_time = 0.
        unloading_start_time = 0.
        waiting_start_time = 0.
        loading_start_time = 0.

        while True:
            if rospy.is_shutdown():
                break

            if self.mode == 0:
                # picker is idle. wait for a row allocation or picking_finished status
                if self.picking_finished:
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.loginfo("%s is idle and at local storage" %(self.picker_id))
                        self.mode = 4
                        unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.loginfo("%s is idle and going to local storage" %(self.picker_id))
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        transportation_start_time = self.env.now

                    # mode changed from idle
                    self.time_spent_idle += self.env.now - idle_start_time

                elif self.curr_row is not None:
                    # idle picker allocated to a new row
                    self.loginfo("%s is allocated to %s" %(self.picker_id, self.curr_row))
                    self.time_spent_idle += self.env.now - idle_start_time
                    self.mode = 1 # transporting to a row_node from curr_node
                    transportation_start_time = self.env.now

                elif self.allocation_finished:
                    # all rows allocated and no assignment for the picker (if there
                    # was any, previous condition would have been satisfied.)
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.loginfo("%s is idle and at local storage" %(self.picker_id))
                        self.mode = 4
                        unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.loginfo("%s is idle and going to local storage" %(self.picker_id))
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        transportation_start_time = self.env.now

                    # mode changed from idle
                    self.time_spent_idle += self.env.now - idle_start_time

                # mode remains idle - not updating self.time_spent_idle now

            elif self.mode == 1:
                # from curr_node go to a row_node (self.goal_node) and continue picking
                # goal_node is set in allocate_row
                self.loginfo("%s going to %s from %s" %(self.picker_id, self.goal_node, self.curr_node))
#                yield self.env.process(self.go_to_node(self.goal_node, self.transportation_rate))
                # adding Gaussian white noise to introduce variations
                yield self.env.process(self.go_to_node(self.goal_node, self.transportation_rate + random.gauss(0, self.transportation_rate_std)))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.loginfo("%s will start picking now" %(self.picker_id))
                self.mode = 2 # picking
                picking_start_time = self.env.now

            elif self.mode == 2:
                # at curr_node, pick through a node_dist to next_node
                yield self.env.process(self.picking_node_to_node())
                self.time_spent_picking += self.env.now - picking_start_time

                if self.curr_row in self.graph.half_rows:
                    # row_end and dir_change nodes are the same
                    row_end_node = self.curr_row_info[2]
                    dir_change_node = self.curr_row_info[2]
                else:
                    row_end_node = self.curr_row_info[1]
                    dir_change_node = self.curr_row_info[2]

                # decide what is the next mode of action
                if self.n_trays >= self.max_n_trays:
                    self.loginfo("%s has trays full" %(self.picker_id))
                    if not self.with_robots:
                        # picker should go to local storage to unload
                        # but, should s/he return?
                        if self.curr_node == row_end_node:
                            # inform row complete and go to local storage node and don't return
                            self.loginfo("%s finished %s" %(self.picker_id, self.curr_row))
                            self.finished_row_routine()
                            self.goal_node = "" + self.local_storage_node
                        elif self.curr_node == dir_change_node:
                            # forward -> reverse
                            self.loginfo("%s changing direction to reverse" %(self.picker_id))
                            self.picking_dir = "reverse"
                            self.goal_node = "" + self.curr_node
                        else:
                            # row not finished, so come back to curr_node and continue in same dir
                            self.loginfo("%s going to %s and will return to %s" %(self.picker_id, self.local_storage_node, self.curr_node))
                            self.goal_node = "" + self.curr_node

                        self.mode = 3
                        transportation_start_time = self.env.now
                    else:
                        if self.curr_node == row_end_node:
                            # inform row complete
                            self.loginfo("%s finished %s" %(self.picker_id, self.curr_row))
                            self.finished_row_routine()
                        elif self.curr_node == dir_change_node:
                            self.loginfo("%s changing direction to reverse" %(self.picker_id))
                            self.picking_dir = "reverse"

                        # go to wait_and_load_on_robot mode (5)
                        self.loginfo("%s waiting for a robot to collect the trays" %(self.picker_id))
                        self.continue_picking = False
                        self.mode = 5
                        self.set_picker_state("CALLED")

                        # wait for the event to register
                        while not rospy.is_shutdown():
                            if self.state == "CALLED" or self.state == "ACCEPT" or self.state == "ARRIVED":
                                break
                            else:
                                yield self.env.timeout(self.loop_timeout)

                        waiting_start_time = self.env.now

                else:
                    # trays not full but has the row finished? if finished, wait for next allocation
                    if self.curr_node == row_end_node:
                        self.loginfo("%s finished %s" %(self.picker_id, self.curr_row))
                        self.finished_row_routine()
                        self.mode = 0
                        idle_start_time = self.env.now

                    elif self.curr_node == dir_change_node:
                        # forward -> reverse, continue picking
                        self.loginfo("%s changing direction to reverse" %(self.picker_id))
                        self.picking_dir = "reverse"
                        self.mode = 2
                        picking_start_time = self.env.now

                    else:
                        # in-between node continue picking
                        self.mode = 2
                        picking_start_time = self.env.now

            elif self.mode == 3:

                storage_node = self.local_storage_node if self.use_local_storage else self.cold_storage_node

                # go to local storage node and change mode to 4
                yield self.env.process(self.go_to_node(storage_node,
                                                       self.transportation_rate))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.mode = 4   # unload at local or cold storage
                unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait and unload at local / cold storage
                # a picker can be in this mode in both cases - with and without robots
                # with robots it could happen only at the end
                # without robots, it could be at the end or when trays are full

                if self.n_trays > 0 or self.picking_progress > 0:
                    if self.use_local_storage:
                        storage = self.graph.local_storages[self.local_storage_node]
                        storage_node = self.local_storage_node
                    else:
                        storage = self.graph.cold_storage
                        storage_node = self.cold_storage_node

                    with storage.request() as req:
                        yield req
                        if not self.with_robots: # without robots
                            if self.picking_finished:
                                self.loginfo("%s unloading all trays at %s" %(self.picker_id, storage_node))
                                wait_time = self.unloading_time * (self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            else:
                                self.loginfo("%s unloading full trays at %s" %(self.picker_id, storage_node))
                                wait_time = self.unloading_time * self.n_trays
                            yield self.env.timeout(wait_time)
                            self.time_spent_unloading += self.env.now - unloading_start_time
                            self.update_trays_unloaded()

                            if self.curr_row is None:
                                # current row is finished. what next?
                                if self.use_local_storage:
                                    # local storage
                                    # if there are no current allocation, stay here
                                    self.mode = 0
                                    idle_start_time = self.env.now
                                else:
                                    # cold storage
                                    if self.allocation_finished:
                                        # all rows allocated stay here
                                        self.mode = 0
                                        idle_start_time = self.env.now
                                    else:
                                        # still unallocated rows, go to local storage of previous
                                        self.mode = 6
                                        transportation_start_time = self.env.now
                            else:
                                # go back to previous node
                                self.mode = 1
                                transportation_start_time = self.env.now

                        else: # with robots
                            # this unloading will happen if s/he finished picking the last allocated row
                            # and tray is not full
                            self.loginfo("%s unloading all trays at %s" %(self.picker_id, storage_node))
                            wait_time = self.unloading_time * (self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            yield self.env.timeout(wait_time)
                            self.time_spent_unloading += self.env.now - unloading_start_time
                            self.update_trays_unloaded()

                            self.mode = 0
                            idle_start_time = self.env.now

                if self.picking_finished or (self.allocation_finished and self.mode == 0):
                    self.loginfo("all rows picked. %s exiting" %(self.picker_id))
                    self.env.exit("all rows picked and idle")
                    break

            elif self.mode == 5:
                # wait for the robot to arrive
                # the car state would change from CALLED -> ACCEPT -> ARRIVED
                # after loading set state to LOADED
                if self.state == "CALLED" or self.state == "ACCEPT":
                    # robot not yet assigned. wait
                    pass
                elif self.state == "ARRIVED":
                    # robot is here. load the full trays and set state as LOADED
                    self.loginfo("%s reached %s" %(self.assigned_robot_id, self.picker_id))
                    self.time_spent_waiting += self.env.now - waiting_start_time

                    # wait for loading on the assigned robot
                    loading_start_time = self.env.now
                    wait_time = self.unloading_time * self.n_trays
                    yield self.env.timeout(wait_time)
                    self.loginfo("%s loaded full trays on %s" %(self.picker_id, self.assigned_robot_id))
                    self.time_spent_loading += self.env.now - loading_start_time

                    self.update_trays_loaded()
                    # set state to LOADED
                    # scheduler should know from the CAR status that the tray is loaded
                    self.set_picker_state("LOADED")
                elif self.state == "INIT":
                    # scheduler knew the robot is loaded and set the state of picker to INIT
                    # the picker is now free to continue picking
                    if self.curr_row is not None:
                        self.loginfo("%s will continue picking %s" %(self.picker_id, self.curr_row))
                        self.mode = 2
                        picking_start_time = self.env.now
                    else:
                        self.loginfo("%s does not have any rows allocated" %(self.picker_id))
                        self.mode = 0
                        idle_start_time = self.env.now

            elif self.mode == 6:
                # go back to previous row's local storage node
                # from curr_node go to local_storage_node and stay idle
                waiting_node = self.prev_row_info[3]
                self.loginfo("%s going to %s from %s" %(self.picker_id, self.goal_node, waiting_node))
                yield self.env.process(self.go_to_node(waiting_node, self.transportation_rate))
                self.time_spent_transportation += self.env.now - transportation_start_time

                self.loginfo("%s is idle now" %(self.picker_id))
                self.mode = 0 # picking
                idle_start_time = self.env.now

                if self.picking_finished or (self.allocation_finished and self.mode == 0):
                    self.loginfo("all rows picked. %s exiting" %(self.picker_id))
                    self.env.exit("all rows picked and idle")
                    break

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)