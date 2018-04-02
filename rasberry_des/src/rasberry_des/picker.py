#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 21/02/2018
# @info: Picker - a simple picker class
# ----------------------------------

import math
import rospy


class Picker(object):
    """Picker class definition"""
    def __init__(self, picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots):
        """Create a Picker object

        Keyword arguments:
        picker_id -- name/id of the picker
        try_capacity -- capacity of the tray picker is carrying
        max_n_trays -- number of trays with the picker
        picking_rate -- rate at which the picker moves while picking
        transportation_rate -- rate at which the picker moves while transporting
        unloading_time -- time the picker will spend at the local storage for unloading
        env -- simpy.Environment
        topo_graph -- TopologicalForkGraph object
        robots -- list of robot agents
        """
        self.picker_id = picker_id
        self.env = env

        self.robots = {}
        self.robot_ids = []
        self.n_robots = len(robots)
        for robot in robots:
            robot_id = robot.robot_id
            self.robot_ids.append(robot_id)
            self.robots[robot_id] = robot

        self.n_rows = 0
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        self.picking_rate = picking_rate
        self.transportation_rate = transportation_rate
        self.max_n_trays = max_n_trays
        self.graph = topo_graph

        # time/tray same for loading on robot or unloading at localStorage
        self.unloading_time = unloading_time

        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        self.mode = 0

        self.picking_finished = False
        self.allocation_finished = False

        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, local_storage_node]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_finish_time = 0. # time at which current row is completed

        self.row_path = []

        self.picking_progress = 0.  # percentage of tray_capacity

        # parameters to check utilisation
        self.time_spent_picking = 0.
        self.time_spent_transportation = 0.
        self.time_spent_idle = 0.
        self.time_spent_waiting = 0.
        self.time_spent_loading = 0.
        self.time_spent_unloading = 0.
        self.time_spent_working = lambda: self.time_spent_picking + self.time_spent_transportation + self.time_spent_unloading

        self.picking_start_time = 0.
        self.transportation_start_time = 0.
        self.idle_start_time = 0.
        self.waiting_start_time = 0.
        self.loading_start_time = 0.
        self.unloading_start_time = 0.

        self.process_timeout = 0.001
        self.loop_timeout = 1.

        self.assigned_robot_id = None

        # TODO: local storage node of the first row is assumed to be the starting loc
        # After reaching another local storage, the robot can wait there
        self.curr_node = self.graph.local_storage_nodes[self.graph.row_ids[0]]

        self.action = self.env.process(self.normal_operation())

    def allocate_row_to_picker(self, row_id):
        """scheduler calls this to assign a row to the picker"""
        assert self.mode == 0
        self.curr_row = row_id
        self.curr_row_info = self.graph.row_info[row_id]
        self.local_storage_node = self.curr_row_info[3]
        self.picking_dir = "forward"
        self.row_path, _, _ = self.graph.get_path_details(self.curr_row_info[1],
                                                          self.curr_row_info[2])
        self.goal_node = "" + self.curr_row_info[1] # goal_node to reach from curr_node

    def assign_robot_to_picker(self, robot_id):
        """scheduler calls this to assign a robot to the picker"""
        assert self.mode == 5
        self.assigned_robot_id = robot_id

    def picking_node_to_node(self, ):
        """mimic picking from one node to another"""
        curr_node_index = self.row_path.index(self.curr_node)
        if self.picking_dir == "forward":
            next_node = self.row_path[curr_node_index + 1]
        elif self.picking_dir == "reverse":
            next_node = self.row_path[curr_node_index - 1]

        start_time = self.env.now
        # move to the next node
        yield self.env.process(self.go_to_node(next_node, self.picking_rate))
        self.time_spent_picking += self.env.now - start_time

        # update picking_progress and n_tray
        self.picking_progress += self.graph.yield_at_node[self.curr_node]
        if self.picking_progress >= self.tray_capacity:
            self.n_trays += 1
            self.picking_progress -= self.tray_capacity

        yield self.env.timeout(self.process_timeout)

    def wait_out(self, wait_time):
        """wait for a given time"""
        start_time = self.env.now
        delta_time = self.env.now - start_time
        while True:
            if rospy.is_shutdown():
                break
            if delta_time >= wait_time:
                break
            else:
                delta_time = self.env.now - start_time
                yield self.env.timeout(self.loop_timeout)
        yield self.env.timeout(self.process_timeout)

    def update_trays_unloaded(self, ):
        """update tray counts after unloading"""
        # update trays unloaded
        self.tot_trays += self.n_trays
        self.n_trays = 0

        if self.picking_finished or self.allocation_finished:
            self.tot_trays += self.picking_progress / self.tray_capacity
            self.picking_progress = 0.

    def dist_to_robot(self, ):
        """return Eucledian distance between robot's pose and picker's pose"""
        robot_node = self.robots[self.assigned_robot_id].curr_node

        if robot_node == self.curr_node:
            return 0

        else:
            robot_node_obj = self.graph.get_node(robot_node)
            robot_x = robot_node_obj.pose.position.x
            robot_y = robot_node_obj.pose.position.y

            curr_node_obj = self.graph.get_node(self.curr_node)
            curr_x = curr_node_obj.pose.position.x
            curr_y = curr_node_obj.pose.position.y

            return math.hypot((robot_x - curr_x), (robot_y - curr_y))

    def update_trays_loaded(self, ):
        """update tray_counts after loading on a robot"""
        self.tot_trays += self.n_trays
        self.n_trays = 0

    def reset_robot_assignment(self, ):
        """reset the assigned robot info after loading"""
        # To ensure the picker won't have a robot assigned, the next time the
        # picker's trays are full and the current robot has not unloaded yet
        self.assigned_robot_id = None

    def normal_operation(self, ):
        """ Picker's picking process when there are robots to carry full trays
        """
        self.idle_start_time = self.env.now

        while True:
            if rospy.is_shutdown():
                break

            rospy.loginfo("%s is in mode %d" %(self.picker_id, self.mode))

            if self.mode == 0:
                # picker is idle. wait for a row allocation or picking_finished status
                print self.picker_id, self.picking_finished, self.curr_row
                if self.picking_finished:
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.mode = 4
                        self.unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        self.transportation_start_time = self.env.now

                    self.time_spent_idle += self.env.now - self.idle_start_time

                elif self.curr_row is not None:
                    self.time_spent_idle += self.env.now - self.idle_start_time
                    self.mode = 1 # transporting to a row_node from curr_node
                    rospy.loginfo("%s is allocated to %s" %(self.picker_id, self.curr_row))

                elif self.allocation_finished:
                    # all rows allocated and no assignment for the picker (if there
                    # was any, previous condition would have been satisfied.)
                    # go to local storage and unload all trays
                    if self.curr_node == self.local_storage_node:
                        # if already at local storage - do nothing
                        self.mode = 4
                        self.unloading_start_time = self.env.now
                    else:
                        # go to local storage node
                        self.goal_node = "" + self.local_storage_node
                        self.mode = 3
                        self.transportation_start_time = self.env.now

                    self.time_spent_idle += self.env.now - self.idle_start_time

            elif self.mode == 1:
                # from curr_node go to a row_node (self.goal_node) and continue picking
                yield self.env.process(self.go_to_node(self.goal_node, self.transportation_rate))
                self.time_spent_transportation += self.env.now - self.transportation_start_time

                self.mode = 2 # picking
                self.picking_start_time = self.env.now

            elif self.mode == 2:
                # at curr_node, pick through a node_dist to next_node
                yield self.env.process(self.picking_node_to_node())
                self.time_spent_picking += self.env.now - self.picking_start_time

                if ((not self.graph.half_rows) and
                        ((self.curr_row == self.graph.row_ids[0]) or
                         (self.curr_row == self.graph.row_ids[-1]))):
                    row_end_node = self.curr_row_info[2]
                    dir_change_node = self.curr_row_info[2]
                else:
                    row_end_node = self.curr_row_info[1]
                    dir_change_node = self.curr_row_info[2]

                # decide what is the next mode of action
                if self.n_trays >= self.max_n_trays:
                    if self.n_robots == 0:
                        # picker should go to local storage to unload
                        # but, should he return?
                        if self.curr_node == row_end_node:
                            # inform row complete and go to local storage node and don't return
                            self.finished_row_routine()
                            self.goal_node = "" + self.local_storage_node
                        elif self.curr_node == dir_change_node:
                            # forward -> reverse
                            self.picking_dir = "reverse"
                            self.goal_node = "" + self.curr_node
                        else:
                            # row not finished, so come back to curr_node and continue in same dir
                            self.goal_node = "" + self.curr_node

                        self.mode = 3
                        self.transportation_start_time = self.env.now
                    else:
                        if self.curr_node == row_end_node:
                            # inform row complete
                            self.finished_row_routine()
                        elif self.curr_node == dir_change_node:
                            self.picking_dir = "reverse"

                        # go to wait_and_load_on_robot mode (5)
                        self.mode = 5
                        self.waiting_start_time = self.env.now

                else:
                    # trays not full but has the row finished? if finished, wait for next allocation
                    if self.curr_node == row_end_node:
                        self.finished_row_routine()
                        self.mode = 0
                        self.idle_start_time = self.env.now

                    elif self.curr_node == dir_change_node:
                        # forward -> reverse, continue picking
                        self.picking_dir = "reverse"
                        self.mode = 2
                        self.picking_start_time = self.env.now

                    else:
                        # in-between node continue picking
                        self.mode = 2
                        self.picking_start_time = self.env.now

            elif self.mode == 3:
                # go to local storage node and change mode to 4
                yield self.env.process(self.go_to_node(self.local_storage_node,
                                                       self.transportation_rate))
                self.time_spent_transportation += self.env.now - self.transportation_start_time

                self.mode = 4
                self.unloading_start_time = self.env.now

            elif self.mode == 4:
                # wait and unload at local storage node
                # a picker can be in this mode in both cases - with and without robots
                # with robots it could happen only at the end
                # without robots, it could be at the end or when trays are full

                if self.n_trays > 0 or self.picking_progress > 0:
                    with self.graph.local_storages[self.local_storage_node].request() as req:
                        yield req
                        if self.n_robots == 0: # without robots
                            if self.picking_finished:
                                wait_time = self.unloading_time * (self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            else:
                                wait_time = self.unloading_time * self.n_trays
                            yield self.env.process(self.wait_out(wait_time))
                            self.time_spent_unloading += self.env.now - self.unloading_start_time
                            self.update_trays_unloaded()

                            # if picking is finished or not, if there are no
                            # current allocations, stay here
                            if self.curr_row is None:
                                self.mode = 0
                                self.idle_start_time = self.env.now
                            else:
                                # go back to previous node
                                self.mode = 1
                                self.transportation_start_time = self.env.now

                        else: # with robots
                            wait_time = self.unloading_time * (self.n_trays if self.picking_progress == 0 else self.n_trays + 1)
                            yield self.env.process(self.wait_out(wait_time))
                            self.time_spent_unloading += self.env.now - self.unloading_start_time
                            self.update_trays_unloaded()

                            self.mode = 0
                            self.idle_start_time = self.env.now

                if self.picking_finished or self.allocation_finished:
                    self.env.exit()
                    break

            elif self.mode == 5:
                # wait for the robot to arrive
                if self.assigned_robot_id is not None:
                    if self.dist_to_robot() == 0:
                        self.time_spent_waiting += self.env.now - self.waiting_start_time
                        # wait for loading on the assigned robot
                        self.loading_start_time = self.env.now
                        wait_time = self.unloading_time * self.n_trays
                        yield self.env.process(self.wait_out(wait_time))
                        self.time_spent_loading += self.env.now - self.loading_start_time
                        self.robots[self.assigned_robot_id].trays_loaded()
                        self.update_trays_loaded()
                        # reset only within this agent, farm monitors whether
                        # the robot completes the unloading
                        self.reset_robot_assignment()

                        if self.curr_row is not None:
                            self.mode = 2
                            self.picking_start_time = self.env.now
                        else:
                            self.mode = 0
                            self.idle_start_time = self.env.now

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

    def go_to_node(self, goal_node, nav_speed):
        """Simpy process to Mimic moving to the goal_node by publishing new position based on speed

        Keyword arguments:
        goal_node -- node to reach from current node
        nav_speed -- navigation speed (picking / transportation rate)
        """
        rospy.loginfo("%s going to %s from %s" %(self.picker_id, goal_node, self.curr_node))
        route_nodes, route_edges, route_distance = self.graph.get_path_details(self.curr_node,
                                                                               goal_node)
        for i in range(len(route_nodes) - 1):
            # move through each edge
            edge_distance = route_distance[i]
            travel_time = edge_distance / nav_speed
            yield self.env.timeout(travel_time)

            self.curr_node = route_nodes[i + 1]

        rospy.loginfo("%s reached %s" %(self.picker_id, goal_node))
        yield self.env.timeout(self.process_timeout)

    def finished_row_routine(self, ):
        """Common things to do when picking along the allocated row is finished
        """
        # some attributes are reset
        self.picking_dir = None
        self.prev_row = "" + self.curr_row
        self.prev_row_info = [] + self.curr_row_info
        self.curr_row = None
        self.row_path = []
        self.row_finish_time = self.env.now
        self.n_rows += 1

    def inform_picking_finished(self, ):
        """called by farm - scheduler to indicate all rows are now picked"""
        self.picking_finished = True

    def inform_allocation_finished(self, ):
        """called by farm - scheduler to indicate all rows are now allocated"""
        self.allocation_finished = True
