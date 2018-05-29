#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import operator
import rospy


class Farm(object):
    """Farm class definition"""
    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        n_topo_nav_rows -- navigational rows in topological map
        topo_map -- TopologicalForkMap object
        robots - robot agent objects
        pickers -- picker agent objects
        policy -- "lexicographical", "shortest_distance", "uniform_utilisation"
        """
        self.name = name
        self.env = env

        self.n_topo_nav_rows = n_topo_nav_rows

        self.pickers = {}
        self.picker_ids = []
        self.n_pickers = len(pickers)
        for picker in pickers:
            self.picker_ids.append(picker.picker_id)
            self.pickers[picker.picker_id] = picker

        self.robots = {}
        self.robot_ids = []
        self.n_robots = len(robots)
        for robot in robots:
            self.robot_ids.append(robot.robot_id)
            self.robots[robot.robot_id] = robot

        # topological map based graph ()
        self.graph = topo_graph

        self.verbose = verbose

        # related to picker finishing a row
        # finished_rows
        self.finished_rows = []
        self.n_finished_rows = 0
        self.row_finish_time = {row_id:None for row_id in self.graph.row_ids} # {row_id: finish time}

        # related to allocation
        self.unallocated_rows = [] + self.graph.row_ids                       # a list of unallocated rows
        self.allocations = {row_id:None for row_id in self.graph.row_ids}     # {row_ids: picker_id}
        self.allocation_time = {row_id:None for row_id in self.graph.row_ids} # {row_id: allocation time}

        self.picker_allocations = {picker_id:[] for picker_id in self.picker_ids}        # {picker_id:[row_ids]}
        self.curr_picker_allocations = {picker_id:None for picker_id in self.picker_ids}   # {picker_id:row_id} row=None if free

        self.process_timeout = 0.001
        self.loop_timeout = 1.

        self.finished_picking = lambda: True if self.n_finished_rows == self.n_topo_nav_rows else False
        self.finished_allocating = lambda: False if self.unallocated_rows else True

        self.idle_pickers = [] + self.picker_ids  # for next assignment
        self.allocated_pickers = [] # for monitoring row_finish
        self.waiting_for_robot_pickers = [] # for assigning a robot to the picker

        self.assigned_picker_robot = {picker_id:None for picker_id in self.picker_ids}
        self.assigned_robot_picker = {robot_id:None for robot_id in self.robot_ids}

        self.idle_robots = [] + self.robot_ids   # for assigning to a picker
        self.assigned_robots = []   # for monitoring transportation progress
        self.charging_robots = []   # ignored for assignments

        self.scheduler_policy = policy

        self.picker_previous_mode = {picker_id:0 for picker_id in self.picker_ids}

        self.action = self.env.process(self.scheduler_monitor())

    def scheduler_monitor(self, ):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        inform_allocation_finished = False
        inform_picking_finished = False
        while True:
            if rospy.is_shutdown():
                break

            if self.finished_picking() and not inform_picking_finished:
                inform_picking_finished = True
                self.loginfo("all rows are picked")
                for robot_id in self.robot_ids:
                    self.robots[robot_id].inform_picking_finished()
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_picking_finished()
                self.loginfo("all rows picked. scheduler exiting")
                self.env.exit("all rows are picked")
                break

            if self.finished_allocating() and not inform_allocation_finished:
                self.loginfo("all rows are allocated")
                inform_allocation_finished = True # do it only once
                for robot_id in self.robot_ids:
                    self.robots[robot_id].inform_allocation_finished()
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_allocation_finished()

            to_remove_pickers = []
            # update modes of pickers already assigned to a row
            for picker_id in self.allocated_pickers:
                picker = self.pickers[picker_id]
                # picker modes
                # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
                # 4: waiting for unload at storage, 5: waiting for loading on robot
                if picker.mode == 0:
                    # finished the assigned row and are idle now
                    # if previously assigned any row, update its status
                    row_id = self.curr_picker_allocations[picker_id]
                    self.finished_rows.append(row_id)
                    self.n_finished_rows += 1
                    self.row_finish_time[row_id] = self.pickers[picker_id].row_finish_time
                    to_remove_pickers.append(picker_id)

                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif picker.mode == 1:
                    # moving to a row_node possibly from the previous node
                    # this can heppen either after a trip to a storage or after a new row allocation
                    # picker will be in picking mode (2) soon
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif picker.mode == 2:
                    # picking now
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif picker.mode == 3 or picker.mode == 4 or picker.mode == 6:
                    # picker transporting to storage (3) or unloading at storage (4)
                    # or transporting to local storage from cold storage (6)
                    # if the current row is finished, the picker's mode will be changed
                    # to idle (0) soon, which will be taken care of in next loop
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif picker.mode == 5:
                    # waiting for a robot to arrive
                    # if a robot is not assigned, assign one
                    if picker_id in self.waiting_for_robot_pickers:
                        # this is an existing request and robot has been assigned or
                        # has not reached yet
                        if self.assigned_picker_robot[picker_id] is not None:
                            # a robot have been already assigned to this picker
                            # and that robot must be on its way
                            # check whether trays are loaded on the robot
                            # if loaded, remove picker from waiting_for_robot_pickers
                            robot_id = self.assigned_picker_robot[picker_id]
                            if self.robots[robot_id].loaded:
                                self.robots[robot_id].proceed_with_transporting()
                                self.pickers[picker_id].proceed_with_picking()
                                self.waiting_for_robot_pickers.remove(picker_id)
                            else:
                                # robot has not reached the picker yet
                                pass
                        else:
                            # no robot has been assigned. scheduler
                            # will try to assign one in this round
                            pass
                    else:
                        if self.assigned_picker_robot[picker_id] is None:
                            # this is a new request for a robot, which should be assigned to this picker
                            self.waiting_for_robot_pickers.append(picker_id)

            for picker_id in to_remove_pickers:
                self.allocated_pickers.remove(picker_id)
                if self.pickers[picker_id].mode == 0:
                    self.idle_pickers.append(picker_id)

            # update prev mode of all pickers
            for picker_id in self.picker_ids:
                picker = self.pickers[picker_id]
                # update previous mode
                if self.picker_previous_mode[picker_id] != picker.mode:
                    self.picker_previous_mode[picker_id] = picker.mode

            # update modes of all assigned robots
            to_remove_robots = []
            for robot_id in self.assigned_robots:
                robot = self.robots[robot_id]
                # robot modes
                # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
                # 3 - waiting for unloading, 4 - transporting to storage, 5- charging
                if robot.mode == 0:
                    # robot completed the unloading at storage, idle now
                    # remove current assignments and add to idle_robots
                    picker_id = self.assigned_robot_picker[robot_id]
                    self.assigned_robot_picker[robot_id] = None
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                    to_remove_robots.append(robot_id)

                elif robot.mode == 1:
                    # transporting to the picker node
                    pass
                elif robot.mode == 2:
                    # waiting for the picker to load
                    picker_id = self.assigned_robot_picker[robot_id]
                    if robot.loaded and picker_id in self.waiting_for_robot_pickers:
                        # trays loaded and scheduler not yet acknowledged
                        picker_id = self.assigned_robot_picker[robot_id]
                        self.robots[robot_id].proceed_with_transporting()
                        self.pickers[picker_id].proceed_with_picking()
                        self.waiting_for_robot_pickers.remove(picker_id)
                    else:
                        # not loaded yet
                        pass
                elif robot.mode == 3:
                    # transporting to local storage
                    picker_id = self.assigned_robot_picker[robot_id]
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif robot.mode == 4:
                    # unloading
                    picker_id = self.assigned_robot_picker[robot_id]
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                elif robot.mode == 5:
                    # charging - after transporting and becoming idle
                    # schdeuler missed the idle mode
                    picker_id = self.assigned_robot_picker[robot_id]
                    self.assigned_robot_picker[robot_id] = None
                    if self.assigned_picker_robot[picker_id] == robot_id:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

                    to_remove_robots.append(robot_id)

                elif robot.mode == 6:
                    # transporting back to local storage after unloading at cold storage
                    picker_id = self.assigned_robot_picker[robot_id]
                    if self.assigned_picker_robot[picker_id] is not None:
                        # if there is a robot assigned to the picker, loading has been completed
                        # remove it from self.assigned_picker_robot[picker_id]
                        self.assigned_picker_robot[picker_id] = None

            # modify assigned_robots list
            for robot_id in to_remove_robots:
                self.assigned_robots.remove(robot_id)
                if self.robots[robot_id].mode == 0:
                    self.idle_robots.append(robot_id)
                elif self.robots[robot_id].mode == 5:
                    self.charging_robots.append(robot_id)

            # charging robots to idle robots
            for robot_id in self.charging_robots:
                robot = self.robots[robot_id]
                if robot.mode == 0:
                    self.charging_robots.remove(robot_id)
                    self.idle_robots.append(robot_id)

            # row allocation to idle_pickers
            self.allocate_rows_to_pickers()

            # assign robots
            if self.n_robots > 0:
                self.assign_robots_to_pickers()

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)

    def allocate_rows_to_pickers(self, ):
        """allocate unallocated_rows to idle_pickers based on scheduler_policy"""
        n_idle_pickers = len(self.idle_pickers)
        if n_idle_pickers > 0:
            allocated_rows = []

            # row allocation is assumed to be "lexicographical"
            self.idle_pickers.sort()
            i = 0
            for row_id in self.unallocated_rows:
                picker_id = self.idle_pickers[0]
                self.allocate(row_id, picker_id)
                allocated_rows.append(row_id)
                if i == n_idle_pickers - 1:
                    break
                else:
                    i += 1

            for row_id in allocated_rows:
                self.unallocated_rows.remove(row_id)

    def get_disatances_from_nodes_dict(self, nodes_dict, goal_node):
        """get distance to a goal_node from nodes given in a dict"""
        node_distances = {}
        for key in nodes_dict:
            _, _, route_distance = self.graph.get_path_details(nodes_dict[key], goal_node)
            node_distances[key] = sum(route_distance)
        return node_distances

    def allocate(self, row_id, picker_id):
        """allocate a picker to a row"""
        # allocate row_id to the picker
        self.allocations[row_id] = picker_id
        self.picker_allocations[picker_id].append(row_id)
        self.curr_picker_allocations[picker_id] = row_id

        self.idle_pickers.remove(picker_id)
        self.allocated_pickers.append(picker_id)

        self.pickers[picker_id].allocate_row_to_picker(row_id)

        self.allocation_time[row_id] = self.env.now

        self.loginfo("%s is allocated to %s at %0.3f" %(picker_id,
                                                        row_id,
                                                        self.allocation_time[row_id]))

    def assign_robots_to_pickers(self, ):
        """assign idle_robots to waiting_for_robot_pickers based on scheduler_policy"""
        n_idle_robots = len(self.idle_robots)
        if n_idle_robots > 0:
            assigned_pickers = []

            # "lexicographical", "shortest_distance", "uniform_utilisation"
            if self.scheduler_policy == "lexicographical":
                # allocate in the order of name
                self.idle_robots.sort()
                i = 0
                for picker_id in self.waiting_for_robot_pickers:
                    if self.pickers[picker_id].assigned_robot_id is None:
                        robot_id = self.idle_robots[0]
                        self.assign(picker_id, robot_id)
                        assigned_pickers.append(picker_id)
                        if i == n_idle_robots - 1:
                            break
                        else:
                            i += 1

            elif self.scheduler_policy == "shortest_distance":
                # allocate in the order of distance to the picker_node

                # max possible allocations is n_idle_robots
                i = 0
                for picker_id in self.waiting_for_robot_pickers:
                    if self.pickers[picker_id].assigned_robot_id is None:
                        # get robot nodes of remaining idle_robots
                        robot_nodes = {}
                        for robot_id in self.idle_robots:
                            robot_nodes[robot_id] = self.robots[robot_id].curr_node

                        picker_node = self.pickers[picker_id].curr_node
                        robot_distances = self.get_disatances_from_nodes_dict(robot_nodes, picker_node)
                        sorted_robots = sorted(robot_distances.items(), key=operator.itemgetter(1))

                        # allocate to the first picker in the sorted
                        robot_id = sorted_robots[0][0]
                        self.assign(picker_id, robot_id)
                        assigned_pickers.append(picker_id)
                        # remove the picker_node from future allocations
                        del robot_nodes[robot_id]
                        # do a max of n_idle_pickers allocations
                        if i == n_idle_robots - 1:
                            break
                        else:
                            i += 1

            elif self.scheduler_policy == "uniform_utilisation":
                # improve utilisation by allocating row to least used picker

                # max possible allocations is n_idle_robots
                i = 0
                for picker_id in self.waiting_for_robot_pickers:
                    if self.pickers[picker_id].assigned_robot_id is None:
                        # get robot utilisation (time_spent_working) of remaining idle_robots
                        robot_utilisation = {}
                        for robot_id in self.idle_robots:
                            robot_utilisation[robot_id] = self.robots[robot_id].time_spent_working()
                        sorted_robots = sorted(robot_utilisation.items(), key=operator.itemgetter(1))

                        # allocate to the first picker in the sorted
                        robot_id = sorted_robots[0][0]
                        self.assign(picker_id, robot_id)
                        assigned_pickers.append(picker_id)
                        # do a max of n_idle_pickers allocations
                        if i == n_idle_robots - 1:
                            break
                        else:
                            i += 1

    def assign(self, picker_id, robot_id):
        """assign a robot to a picker waiting will full trays"""
        self.idle_robots.remove(robot_id)
        self.assigned_robots.append(robot_id)
        picker_node = self.pickers[picker_id].curr_node
        picker_n_trays = self.pickers[picker_id].n_trays
        picker_local_storage_node = self.pickers[picker_id].local_storage_node

        self.assigned_picker_robot[picker_id] = robot_id
        self.assigned_robot_picker[robot_id] = picker_id
        self.pickers[picker_id].assign_robot_to_picker(robot_id)
        self.robots[robot_id].assign_robot_to_picker(picker_id, picker_node,
                                                     picker_n_trays,
                                                     picker_local_storage_node)

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
