#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.farm


class FarmMimic(rasberry_des.farm.Farm):
    """Farm class definition"""
    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, n_iters=0, verbose=False):
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
        super(FarmMimic, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)
        self.n_iters = n_iters
        self.curr_iter = 0
        self.curr_picker_iter = {picker_id:self.curr_iter for picker_id in self.picker_ids}
        self.curr_row_iter = {row_id:self.curr_iter for row_id in self.graph.row_ids}


    def check_iter_increment(self, ):
        """checks whether incrementing the iteration number for all rows is possible
        """
        # increment all idle picker's iter by one
        for picker_id in self.picker_ids:
            if self.pickers[picker_id].curr_mode == 0 and self.curr_picker_iter[picker_id] < self.curr_iter:
                self.curr_picker_iter += 1

        # This is possible when all
        pass

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
                    # picker transporting to storage or unloading at storage
                    # or transporting to local storage from cold storage
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

            # update modes of all assigned robots
            to_remove_robots = []
            for robot_id in self.assigned_robots:
                robot = self.robots[robot_id]
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