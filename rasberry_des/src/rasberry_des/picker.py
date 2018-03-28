#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 21/02/2018
# @info: Picker - a simple picker class
# ----------------------------------

import math
import rospy
import tf
import geometry_msgs.msg
import rasberry_des.msg
import rasberry_des.srv


class Picker(object):
    """Picker class definition"""
    def __init__(self, picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, farm, topo_graph, des_env, robots, sim_rt_factor=1.0):
        """Create a Picker object

        Keyword arguments:
        picker_id -- name/id of the picker
        env -- simpy.Environment
        farm -- provides the allocation and monitor process as well as a graph.
        try_capacity -- capacity of the tray picker is carrying
        max_n_trays -- number of trays with the picker
        picking_rate -- rate at which the picker moves while picking
        transportation_rate -- rate at which the picker moves while transporting
        unloading_time -- time the picker will spend at the local storage for unloading
        """
        self.picker_id = picker_id
        self.env = env
        self.farm = farm
        self.robots = {}
        self.robot_ids = []
        for robot in robots:
            robot_id = robot.robot_id
            self.robots[robot_id] = robot
        self.n_rows = 0
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        if des_env == "simpy":
            sim_rt_factor = 1.0 # ignore sim_rt_factor for "ros" env
        self.picking_rate = picking_rate * sim_rt_factor
        self.transportation_rate = transportation_rate * sim_rt_factor
        self.max_n_trays = max_n_trays
        self.graph = topo_graph

        self.unloading_time = unloading_time / sim_rt_factor    # time spent at localStorage

        self.mode = 0       # 0:free, 1:picking, 2:transporting, 3:finished_job
        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, row_node_dist, last_node_dist]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_path = []

        self.picking_progress = 0.  # percentage of tray_capacity

        # parameters to check utilisation
        self.time_spent_picking = 0.
        self.time_spent_transportation = 0.
        self.time_spent_idle = 0.
        self.time_spent_loading = 0.
        self.time_spent_unloading = 0.

        if des_env == "simpy":
            self.pub_delay = 1.0
            self.process_timeout = 0.001
            self.loop_timeout = 1.0
        elif des_env == "ros":
            self.pub_delay = max(0.25, 0.25 / sim_rt_factor)
            self.process_timeout = 0.001
            self.loop_timeout = 0.1

        # services / clients
        # client of farm service - trays_full
        rospy.loginfo("%s waiting for %s service" %(self.picker_id, "trays_full"))
        rospy.wait_for_service("trays_full")
        self.trays_full_client = rospy.ServiceProxy("trays_full", rasberry_des.srv.Trays_Full)
        rospy.loginfo("%s conencted to %s service" %(self.picker_id, "trays_full"))
        self.trays_full_request = rasberry_des.srv.Trays_FullRequest()
        self.trays_full_request.picker_id = self.picker_id

        # client of farm service - trays_unloaded
        rospy.loginfo("%s waiting for %s service" %(self.picker_id, "trays_unloaded"))
        rospy.wait_for_service("trays_unloaded")
        self.trays_unloaded_client = rospy.ServiceProxy("trays_unloaded", rasberry_des.srv.Trays_Full)
        rospy.loginfo("%s conencted to %s service" %(self.picker_id, "trays_unloaded"))
        self.trays_unloaded_request = rasberry_des.srv.Trays_FullRequest()
        self.trays_unloaded_request.picker_id = self.picker_id

        # client of farm service - robot_info
        rospy.loginfo("%s waiting for %s service" %(self.picker_id, "robot_info"))
        rospy.wait_for_service("robot_info")
        self.robot_info_client = rospy.ServiceProxy("robot_info", rasberry_des.srv.Robot_Info)
        rospy.loginfo("%s conencted to %s service" %(self.picker_id, "robot_info"))
        self.robot_info_request = rasberry_des.srv.Robot_InfoRequest()
        self.robot_info_request.picker_id = self.picker_id

        if self.farm.n_robots == 0:
            self.action = self.env.process(self.pickers_without_robots())
        else:
            self.action = self.env.process(self.pickers_with_robots())

    def init_status_attribute(self, picking_rate, transportation_rate):
        self.status.picker_id = self.picker_id
        self.status.picking_rate = picking_rate
        self.status.transportation_rate = transportation_rate
        self.status.tray_capacity = self.tray_capacity
        self.status.picking_progress = 0.
        self.status.n_trays = 0
        self.status.tot_trays = 0
        self.status.n_rows = 0
        self.status.curr_row = "None"
        self.status.mode = self.mode

    def pickers_with_robots(self, ):
        """ Picker's picking process when there are robots to carry full trays
        """
        # 1. picker should report for duty first
        self.farm.picker_report(self.picker_id)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]

        while True:
            # 2. If the picker is assigned a row,
            #   a. continue picking
            #   b. yield go_to_node(next_node) process
            #   c. update and check tray_cap,
            #       i. if tray_cap is reached, increment n_trays
            #       ii. if n_trays reach max_n_trays, start the transport_process
            #       ii. if not move to the next row_node in the next iter
            if self.mode == 1:
                # move along each node in the assigned row
                # the picker is at the curr_node
                # move to the node next to curr_node in the row_path
                if self.picking_dir is "forward":
                    curr_node_index = self.row_path.index(self.curr_node)
                    next_node = self.row_path[curr_node_index + 1]
                    # pick through to the next node
                    yield self.env.process(self.go_to_node(next_node, self.picking_rate))

                    # update the picking progress
                    self.picking_progress += self.graph.yield_at_node[self.curr_node]

                    # reverse at the end
                    if self.curr_node == self.row_path[-1]:
                        self.picking_dir = "reverse"
                        rospy.loginfo("%s changing to reverse along %s at %0.3f" %(self.picker_id,
                                                                                   self.curr_row,
                                                                                   self.env.now))

                    # if the tray capacity is reached, increment n_trays
                    if self.picking_progress >= self.tray_capacity:
                        self.n_trays += 1
                        self.picking_progress -= self.tray_capacity

                    # if max_n_trays is reached
                    if self.n_trays == self.max_n_trays:
                        # inform farm about trays_full, robot_info, trays_unloaded
                        yield self.env.process(self.load_on_robot())

                        # if full rows, and at first or last row, and at the end node,
                        #   send row finished. wait for any new row
                        if ((not self.graph.half_rows) and
                                ((self.curr_row == self.graph.row_ids[0]) or
                                 (self.curr_row == self.graph.row_ids[-1])) and
                                (self.curr_node == self.row_path[-1])):
                            # row is finished
                            self.finished_row_routine()
                            # no current allocation - change mode back to zero
                            self.mode = 0

                elif self.picking_dir is "reverse":
                    # work with negative indices
                    curr_node_index = self.row_path.index(self.curr_node) - len(self.row_path)
                    next_node = self.row_path[curr_node_index - 1]

                    if ((not self.graph.half_rows) and
                            ((self.curr_row == self.graph.row_ids[0]) or
                             (self.curr_row == self.graph.row_ids[-1])) and
                            (self.curr_node == self.row_path[-1])):
                        # there are full rows at the start and end
                        # row is finished
                        # navigate to the start node of the row and send row finish (no picking)
                        next_node = self.row_path[0] # finished_row_routine will reset row_path
                        self.finished_row_routine()
                        yield self.env.process(self.go_to_node(next_node, self.transportation_rate))
                    else:
                        # half rows at start and end (berries on both sides)
                        # pick through to the next node
                        yield self.env.process(self.go_to_node(next_node, self.picking_rate))
                        # update the picking progress
                        self.picking_progress += self.graph.yield_at_node[self.curr_node]
                        if self.curr_node == self.row_path[0]:
                            # row is finished
                            self.finished_row_routine()

                    # check picking progress
                    if self.picking_progress >= self.tray_capacity:
                        self.picking_progress -= self.tray_capacity
                        self.n_trays += 1

                        if self.n_trays == self.max_n_trays:
                            # inform farm about trays_full, robot_info, trays_unloaded
                            yield self.env.process(self.load_on_robot())

                            if self.curr_row is None:
                                # finished picking along curr_row
                                # reset mode to no allocations
                                self.mode = 0

            # 3. If in mode free, check if there is any new assignments
            #       If there is no new assignment and no rows left to be assigned, finish picking
            #       If there is a new assignment
            #           a. Move to the start node of the path
            #           b. Get the path from the current loc to start_node of new row
            #           c. Move at transportation_rate to start_node
            #           d. Continue/Start picking along the new row by changing mode to picking
            elif self.mode == 0:
                row_id = self.farm.curr_picker_allocations[self.picker_id]
                row_id = None if row_id == self.prev_row else row_id

                if (row_id is None) and (len(self.farm.unallocated_rows) == 0):
                    # The picker has an assigned row and all rows are allocated
                    # unload any berries left in hand and leave the picking process
                    if self.curr_node == self.local_storage_node:
                        # at local storage after unloading max_n_trays
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.unload(item="all"))
                    elif self.curr_node is not None:
                        # the picker is at some node already
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.transport_to_local_storage(item="all"))
                    self.mode = 3
                    # finish the picking process
                    rospy.loginfo("%s finishing picking process. all rows are assigned" %(self.picker_id))
                    break

                elif row_id is not None: # if there is a row assigned to the picker
                    self.curr_row = row_id
                    self.curr_row_info = self.graph.row_info[self.curr_row]
                    # TODO: Now the local_storage_node of the first assigned row is assumed to be
                    # the starting position of the picker. Is an origin_node required?
                    if self.curr_node is None:
                        self.curr_node = self.graph.local_storage_nodes[self.curr_row]
                        self.local_storage_node = self.graph.local_storage_nodes[self.curr_row]

                    rospy.loginfo("%s is moving to the start of %s at %0.3f" %(self.picker_id,
                                                                               self.curr_row,
                                                                               self.env.now))
                    self.mode = 2
                    # go to the start_node of the row
                    yield self.env.process(self.go_to_node(self.curr_row_info[1], self.transportation_rate))

                    # picker moved to the start_node of the row (yield above)
                    # get the path from start to end of the row
                    self.row_path, _, _ = self.graph.get_path_details(self.curr_node, self.curr_row_info[2])

                    rospy.loginfo("%s started forward picking on %s at %0.3f" %(self.picker_id, row_id, self.env.now))
                    # change current mode to picking
                    self.mode = 1 # picking mode

                    self.picking_dir = "forward"

            yield self.env.timeout(self.process_timeout)

    def pickers_without_robots(self, ):
        """ Picker's picking process
        """
        # 1. picker should report for duty first
        self.farm.picker_report(self.picker_id)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]

        while True:
            # 2. If the picker is assigned a row,
            #   a. continue picking
            #   b. yield go_to_node(next_node) process
            #   c. update and check tray_cap,
            #       i. if tray_cap is reached, increment n_trays
            #       ii. if n_trays reach max_n_trays, start the transport_process
            #       ii. if not move to the next row_node in the next iter
            if self.mode == 1:
                # move along each node in the assigned row
                # the picker is at the curr_node
                # move to the node next to curr_node in the row_path
                if self.picking_dir is "forward":
                    curr_node_index = self.row_path.index(self.curr_node)
                    next_node = self.row_path[curr_node_index + 1]
                    # pick through to the next node
                    yield self.env.process(self.go_to_node(next_node, self.picking_rate))

                    # update the picking progress
                    self.picking_progress += self.graph.yield_at_node[self.curr_node]

                    # reverse at the end
                    if self.curr_node == self.row_path[-1]:
                        self.picking_dir = "reverse"
                        rospy.loginfo("%s changing to reverse along %s at %0.3f" %(self.picker_id,
                                                                                   self.curr_row,
                                                                                   self.env.now))

                    # if the tray capacity is reached, increment n_trays
                    if self.picking_progress >= self.tray_capacity:
                        self.n_trays += 1
                        self.picking_progress -= self.tray_capacity

                    # if max_n_trays is reached
                    if self.n_trays == self.max_n_trays:
                        self.trays_full_request.curr_node = self.curr_node
                        self.trays_full_request.n_trays = self.n_trays
                        self.trays_full_client(self.trays_full_request)
                        # if full rows, and at first or last row, and at the end node,
                        #   send row finished
                        #   go to local storage and no return
                        if ((not self.graph.half_rows) and
                                ((self.curr_row == self.graph.row_ids[0]) or
                                 (self.curr_row == self.graph.row_ids[-1])) and
                                (self.curr_node == self.row_path[-1])):
                            # row is finished
                            self.finished_row_routine()
                            # transport to the local storage and don't return
                            yield self.env.process(self.transport_to_local_storage(item="tray"))
                            # finished the allocated row and transported full berry trays
                            # now at local_storage_node
                            # no current allocation - change mode back to zero
                            self.mode = 0
                        else:
                            curr_node = "" + self.curr_node # back up of self.curr_node
                            yield self.env.process(self.transport_to_local_storage(item="tray"))
                            # now at local storage, return to the curr_node
                            yield self.env.process(self.go_to_node(curr_node, self.transportation_rate))
                            # resume picking
                            self.mode = 1

                elif self.picking_dir is "reverse":
                    # work with negative indices
                    curr_node_index = self.row_path.index(self.curr_node) - len(self.row_path)
                    next_node = self.row_path[curr_node_index - 1]

                    if ((not self.graph.half_rows) and
                            ((self.curr_row == self.graph.row_ids[0]) or
                             (self.curr_row == self.graph.row_ids[-1])) and
                            (self.curr_node == self.row_path[-1])):
                        # there are full rows at the start and end
                        # row is finished
                        # navigate to the start node of the row and send row finish (no picking)
                        next_node = self.row_path[0] # finished_row_routine will reset row_path
                        self.finished_row_routine()
                        yield self.env.process(self.go_to_node(next_node, self.transportation_rate))
                    else:
                        # half rows at start and end (berries on both sides)
                        # pick through to the next node
                        yield self.env.process(self.go_to_node(next_node, self.picking_rate))
                        # update the picking progress
                        self.picking_progress += self.graph.yield_at_node[self.curr_node]
                        if self.curr_node == self.row_path[0]:
                            # row is finished
                            self.finished_row_routine()

                    # check picking progress
                    if self.picking_progress >= self.tray_capacity:
                        self.picking_progress -= self.tray_capacity
                        self.n_trays += 1

                        if self.n_trays == self.max_n_trays:
                            self.trays_full_request.curr_node = self.curr_node
                            self.trays_full_request.n_trays = self.n_trays
                            self.trays_full_client(self.trays_full_request)

                            if self.curr_row is None:
                                # finished picking along curr_row
                                # transport to local storage and don't return
                                yield self.env.process(self.transport_to_local_storage(item="tray"))
                                # reset mode to no allocations
                                self.mode = 0
                            else:
                                # transport to local storage and return
                                curr_node = "" + self.curr_node
                                yield self.env.process(self.transport_to_local_storage(item="tray"))
                                # return
                                yield self.env.process(self.go_to_node(curr_node, self.transportation_rate))
                                # resume picking
                                self.mode = 1

            # 3. If in mode free, check if there is any new assignments
            #       If there is no new assignment and no rows left to be assigned, finish picking
            #       If there is a new assignment
            #           a. Move to the start node of the path
            #           b. Get the path from the current loc to start_node of new row
            #           c. Move at transportation_rate to start_node
            #           d. Continue/Start picking along the new row by changing mode to picking
            elif self.mode == 0:
                row_id = self.farm.curr_picker_allocations[self.picker_id]
                row_id = None if row_id == self.prev_row else row_id

                if (row_id is None) and (len(self.farm.unallocated_rows) == 0):
                    # The picker has an assigned row and all rows are allocated
                    # unload any berries left in hand and leave the picking process
                    if self.curr_node == self.local_storage_node:
                        # at local storage after unloading max_n_trays
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.unload(item="all"))
                    elif self.curr_node is not None:
                        # the picker is at some node already
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.transport_to_local_storage(item="all"))
                    self.mode = 3
                    # finish the picking process
                    rospy.loginfo("%s finishing picking process. all rows are assigned" %(self.picker_id))
                    break

                elif row_id is not None: # if there is a row assigned to the picker
                    self.curr_row = row_id
                    self.curr_row_info = self.graph.row_info[self.curr_row]
                    # TODO: Now the local_storage_node of the first assigned row is assumed to be
                    # the starting position of the picker. Is an origin_node required?
                    if self.curr_node is None:
                        self.curr_node = self.graph.local_storage_nodes[self.curr_row]
                        self.local_storage_node = self.graph.local_storage_nodes[self.curr_row]

                    rospy.loginfo("%s is moving to the start of %s at %0.3f" %(self.picker_id,
                                                                               self.curr_row,
                                                                               self.env.now))
                    self.mode = 2
                    # go to the start_node of the row
                    yield self.env.process(self.go_to_node(self.curr_row_info[1], self.transportation_rate))

                    # picker moved to the start_node of the row (yield above)
                    # get the path from start to end of the row
                    self.row_path, _, _ = self.graph.get_path_details(self.curr_node, self.curr_row_info[2])

                    rospy.loginfo("%s started forward picking on %s at %0.3f" %(self.picker_id, row_id, self.env.now))
                    # change current mode to picking
                    self.mode = 1 # picking mode

                    self.picking_dir = "forward"

            yield self.env.timeout(self.process_timeout)

    def load_on_robot(self, ):
        """Picker's unloading process at the local storage node

        Keyword arguments:

        item -- unload "tray" or "all"; "tray" is normal, "all" only when no more rows are free
        """
        # inform farm that trays are full
        self.trays_full_request.n_trays = self.n_trays
        self.trays_full_request.curr_node = self.curr_node
        self.trays_full_client(self.trays_full_request)
        rospy.loginfo("%s informed scheduler about trays_full" %(self.picker_id))
        # request for the allocated robot info
        robot_id = self.robot_info_client(self.robot_info_request).robot_id
        rospy.loginfo("%s is assigned to %s" %(self.picker_id, robot_id))

        # wait for the robot to arrive
        # TODO: try to check with curr_node instead of dist < 0
        while self.dist_to_robot(robot_id) > 0:
            rospy.loginfo("distance between %s and %s: %0.1f" %(self.picker_id, robot_id, self.dist_to_robot(robot_id)))
            yield self.env.timeout(self.loop_timeout)

        # If the robot is at the current node, wait for unloading time
        wait_time = self.unloading_time * self.n_trays
        yield self.env.timeout(wait_time)

        # call trays_unloaded service
        # The farm will in turn call the robot service to indicate the loading on robot is complete
        self.trays_unloaded_request.n_trays = self.max_n_trays
        self.trays_unloaded_request.curr_node = self.curr_node
        self.tot_trays += self.max_n_trays
        self.n_trays -= self.max_n_trays
        self.trays_unloaded_client(self.trays_unloaded_request)

        rospy.loginfo("%s finished loading trays on %s at %0.1f" %(self.picker_id, robot_id, self.env.now))
        rospy.loginfo("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.picker_id,
                                                                                    self.tot_trays,
                                                                                    self.n_trays,
                                                                                    self.picking_progress))

        yield self.env.timeout(self.process_timeout)

    def dist_to_robot(self, robot_id):
        """return Eucledian distance between robot's pose and picker's pose"""
        robot_node = self.robots[robot_id].curr_node
        robot_node_obj = self.graph.get_node(robot_node)
        robot_x = robot_node_obj.pose.position.x
        robot_y = robot_node_obj.pose.position.y

        curr_node_obj = self.graph.get_node(self.curr_node)
        curr_x = curr_node_obj.pose.position.x
        curr_y = curr_node_obj.pose.position.y

        return math.hypot((robot_x - curr_x), (robot_y - curr_y))

    def unload(self, item="tray"):
        """Picker's unloading process at the local storage node

        Keyword arguments:

        item -- unload "tray" or "all"; "tray" is normal, "all" only when no more rows are free
        """
        rospy.loginfo("%s requesting for accessing the local storage %s at %0.1f" %(self.picker_id, self.local_storage_node, self.env.now))
        with self.graph.local_storages[self.local_storage_node].request() as req:
            # wait for permission to acces local storage, no pose publishing in between
            yield req
            rospy.loginfo("%s was granted access to local storage %s at %0.1f" %(self.picker_id, self.local_storage_node, self.env.now))
            unloading_time = self.unloading_time * (self.n_trays if item == "tray" else (self.n_trays + 1))
            yield self.env.timeout(unloading_time)

        # update tot_trays
        if item == "tray":
            self.trays_unloaded_request.n_trays = self.max_n_trays
            self.trays_unloaded_request.curr_node = self.curr_node
            self.tot_trays += self.max_n_trays
            self.n_trays -= self.max_n_trays
            self.trays_unloaded_client(self.trays_unloaded_request)

        elif item == "all":
            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
            self.n_trays = 0
            self.picking_progress = 0
            self.mode = 0

        rospy.loginfo("%s finished unloading at %0.1f" %(self.picker_id, self.env.now))
        rospy.loginfo("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.picker_id,
                                                                                    self.tot_trays,
                                                                                    self.n_trays,
                                                                                    self.picking_progress))

        yield self.env.timeout(self.process_timeout)

    def go_to_node(self, goal_node, nav_speed):
        """Simpy process to Mimic moving to the goal_node by publishing new position based on speed

        Keyword arguments:
        goal_node -- node to reach from current node
        nav_speed -- navigation speed (picking / transportation rate)
        """
        route_nodes, route_edges, route_distance = self.graph.get_path_details(self.curr_node,
                                                                                    goal_node)
        for i in range(len(route_nodes) - 1):
            # move through each edge
            edge_distance = route_distance[i]
            travel_time = edge_distance / nav_speed
            yield self.env.timeout(travel_time)

            self.curr_node = route_nodes[i + 1]

        yield self.env.timeout(self.process_timeout)

    def transport_to_local_storage(self, item="tray"):
        """Transport item to local storage by yielding to go_to_node and unload processes

        Keyword arguments:

        item -- unload "tray" or "all"; "tray" is normal, "all" only when no more rows are free
        """
        # transport to the local storage and don't return
        self.mode = 2
        rospy.loginfo("%s reached %d trays. going to local storage at %0.3f" %(self.picker_id,
                                                                               self.max_n_trays,
                                                                               self.env.now))
        yield self.env.process(self.go_to_node(self.local_storage_node, self.transportation_rate))

        # reached local storage, now unload
        yield self.env.process(self.unload(item))

    def finished_row_routine(self, ):
        """Common things to do when picking along the allocated row is finished
        """
        # trigger finished_rows event for the curr_row
        self.farm.finished_rows[self.curr_row].succeed(value=self.env.now)
        # some attributes are reset
        self.picking_dir = None
        self.prev_row = "" + self.curr_row
        self.prev_row_info = [] + self.curr_row_info
        self.curr_row = None
        self.row_path = []
        self.n_rows += 1
        self.mode = 0
