#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import simpy
import rospy
import geometry_msgs.msg
import rasberry_des.msg
import actionlib


class Farm(object):
    """Farm class definition"""
    def __init__(self, name, env, des_env, n_topo_nav_rows, topo_graph, picker_ids, robot_ids, sim_rt_factor=1.0):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        """
        self.name = name
        self.env = env

        # pickers should report for duty.
        # could be useful in future to take breaks or late arrivals
        self.pickers_reported = []

        self.n_topo_nav_rows = n_topo_nav_rows
        self.picker_ids = picker_ids
        self.n_pickers = len(self.picker_ids)
        self.robot_ids = robot_ids
        self.n_robots = len(self.robot_ids)

        # topological map based graph ()
        self.graph = topo_graph

        # related to picker finishing a row
        # finished_rows - A dict of simpy.Events. picker triggers
        self.finished_rows = {row_id:simpy.Event(self.env) for row_id in self.graph.row_ids}
        self.n_finished_rows = 0
        self.row_finish_time = {row_id:None for row_id in self.graph.row_ids} # {row_id: finish time}

        # related to allocation
        self.unallocated_rows = [] + self.graph.row_ids                       # a list of unallocated rows
        self.allocations = {row_id:None for row_id in self.graph.row_ids}     # {row_ids: picker_id}
        self.allocation_time = {row_id:None for row_id in self.graph.row_ids} # {row_id: allocation time}

        self.picker_allocations = {}        # {picker_id:[row_ids]}
        self.curr_picker_allocations = {}   # {picker_id:row_id} row=None if free

        # services / clients
        self.trays_full_service = rospy.Service("trays_full", rasberry_des.srv.Trays_Full, self.trays_full)
        rospy.loginfo("farm initialised service 'trays_full'")
        self.trays_unloaded_service = rospy.Service("trays_unloaded", rasberry_des.srv.Trays_Full, self.trays_unloaded)
        rospy.loginfo("farm initialised service 'trays_unloaded'")
        self.robot_info_service = rospy.Service("robot_info", rasberry_des.srv.Robot_Info, self.send_robot_info)
        rospy.loginfo("farm initialised service 'robot_info'")

        self.trays_full_response = rasberry_des.srv.Trays_FullResponse()
        self.trays_unloaded_response = rasberry_des.srv.Trays_FullResponse()
        self.robot_info_response = rasberry_des.srv.Robot_InfoResponse()

        self.robot_trays_loaded_clients = {}
        for robot_id in self.robot_ids:
            self.robot_trays_loaded_clients[robot_id] = rospy.ServiceProxy("%s/trays_loaded" %(robot_id), rasberry_des.srv.Trays_Full)
            rospy.loginfo("%s waiting for %s/trays_loaded service" %("farm", robot_id))
            rospy.wait_for_service("%s/trays_loaded" %(robot_id))
            rospy.loginfo("farm connected to service %s/trays_loaded" %(robot_id))

        # action client
        self.robot_collection_clients = {}
        for robot_id in self.robot_ids:
            self.robot_collection_clients[robot_id] = actionlib.SimpleActionClient("%s/collection" %(robot_id),
                                                                                   rasberry_des.msg.Robot_CollectionAction)
            self.robot_collection_clients[robot_id].wait_for_server()

        self.assigned_picker_robot = {picker_id:None for picker_id in self.picker_ids}
        self.assigned_robot_picker = {robot_id:None for robot_id in self.robot_ids}

        self.trays_full_pickers = []
        self.trays_full_picker_n_trays = {}
        self.trays_full_picker_nodes = {}
        self.tot_trays_unloaded = 0.0

        if des_env == "simpy":
            self.pub_delay = 1.0
            self.process_timeout = 0.001
            self.loop_timeout = 1.0
        elif des_env == "ros":
            self.pub_delay = max(0.25, 0.25 / sim_rt_factor)
            self.process_timeout = 0.001
            self.loop_timeout = 0.1

    def send_robot_info(self, srv):
        """After sending the tarys_full service request, a picker will request for robot info.
        These are decoupled to enable scheduling outside the trays_full service response.
        If the picker is in the tray full list, a while loop is run until a robot is assigned.
        The assigned robot's id is sent as response to the picker
        """
        self.robot_info_response.robot_id = None
        robot_assigned = False
        while not robot_assigned:
            for robot_id in self.robot_ids:
                if self.assigned_robot_picker[robot_id] is None:
                    self.assign_robot_to_picker(robot_id, srv.picker_id) # by sending action goal to the robot
                    self.robot_info_response.robot_id = robot_id
                    robot_assigned = True
                    break
            rospy.sleep(0.001)
        rospy.loginfo("%s, %s" %(srv.picker_id, self.robot_info_response.robot_id))
        return self.robot_info_response

    def trays_full(self, srv):
        """callback function for trays_full service"""
        self.trays_full_pickers.append(srv.picker_id)
        self.trays_full_picker_n_trays[srv.picker_id] = srv.n_trays
        self.trays_full_picker_nodes[srv.picker_id] = srv.curr_node
        return self.trays_full_response

    def trays_unloaded(self, srv):
        """callback function for trays_unloaded service"""
        # picker is calling this only from row nodes
        if self.n_robots > 0:
            # if there is a robot assigned to the picker, call its trays_loaded service
            robot_id = self.assigned_picker_robot[srv.picker_id]
            request = rasberry_des.srv.Trays_FullRequest()
            request.picker_id = srv.picker_id
            request.n_trays = srv.n_trays
            self.robot_trays_loaded_clients[robot_id](request)
        else:
            self.update_tray_counts(srv.picker_id, srv.n_trays)
        return self.trays_unloaded_response

    def update_tray_counts(self, picker_id, n_trays):
        """update tray counts after unloading"""
        self.tot_trays_unloaded += n_trays
        self.trays_full_pickers.remove(picker_id)
        self.trays_full_picker_n_trays[picker_id] = 0

    def picker_report(self, picker_id):
        """Method a picker should call when he reports to work

        Keyword arguments:
        picker_id -- picker's unique id
        """
        if picker_id not in self.pickers_reported:
            self.picker_allocations[picker_id] = []
            self.curr_picker_allocations[picker_id] = None
            self.pickers_reported.append(picker_id)
            rospy.loginfo("%s reporting at %0.3f" %(picker_id, self.env.now))

    def finished_picking(self, ):
        """Method to check whether all allocated rows are finished"""
        # method to check whether all rows are picked.
        # return True or False, based on picking is finished or not
        if self.n_finished_rows == self.n_topo_nav_rows:
            return True
        return False

    def finished_allocating(self, ):
        """Method to check whether all rows are allocated"""
        # method to check whether all rows are allocated.
        if self.unallocated_rows:
            return False
        return True

    def assign_robot_to_picker(self, robot_id, picker_id):
        """Assign a robot to go to a picker, collect full trays and take them to storage"""
        collection_goal = rasberry_des.msg.Robot_CollectionGoal()
        collection_goal.picker_id = picker_id
        collection_goal.picker_node = self.trays_full_picker_nodes[picker_id]
        collection_goal.local_storage_node = self.graph.local_storage_nodes[self.curr_picker_allocations[picker_id]]
        self.assigned_picker_robot[picker_id] = robot_id
        self.assigned_robot_picker[robot_id] = picker_id
        self.robot_collection_clients[robot_id].send_goal(collection_goal, done_cb=self.collection_done)

    def collection_done(self, state, result):
        robot_id = result.robot_id
        picker_id = self.assigned_robot_picker[robot_id]
        n_trays = self.trays_full_picker_n_trays[picker_id]
        self.update_tray_counts(picker_id, n_trays)
        self.assigned_picker_robot[picker_id] = None
        self.assigned_robot_picker[robot_id] = None

    def scheduler_monitor(self, ):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            external: picker triggers the finishedRows[row_id] event,
                        when the allocated row is completed
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        ns = rospy.get_namespace()
        while True:
            # when there are robots, assign them to collect
            if self.n_robots > 0:
                # assign each request to a robot by calling its Robot_Collection action
                # TODO: This is done in robot_info_service request
                pass

            # check for any completion update of any rows
            for i in range(len(self.pickers_reported)):
                picker_id = self.pickers_reported[i]
                if self.curr_picker_allocations[picker_id] is not None:
                    # check if there is a row allocated to the picker
                    row_id = self.curr_picker_allocations[picker_id]
                    if self.finished_rows[row_id].triggered:
                        # this row is finished
                        self.n_finished_rows += 1
                        # TODO: there could be slight delay in this time
                        rospy.loginfo("%s reported finish-row at %0.3f and now time is %0.3f" %(picker_id,
                                                                                                self.finished_rows[row_id].value,
                                                                                                self.env.now))
                        self.row_finish_time[row_id] = self.finished_rows[row_id].value
                        # relieve the picker
                        self.curr_picker_allocations[picker_id] = None
                        rospy.loginfo("%s reported completion of row %s at %0.3f" %(picker_id,
                                                                                    row_id,
                                                                                    self.row_finish_time[row_id]))
            if self.finished_picking():
                # once all rows are picked finish the process
                rospy.set_param(ns + "des_running", False)
                break

            # allocate, if there are rows yet to be allocated
            if not self.finished_allocating():
                for picker_id in self.pickers_reported:
                    if not self.curr_picker_allocations[picker_id]:
                        # allocate if picker is free
                        # get the first free row
                        row_id = self.unallocated_rows.pop(0)
                        # allocate row_id to the picker
                        self.allocations[row_id] = picker_id
                        self.picker_allocations[picker_id].append(row_id)
                        self.allocation_time[row_id] = self.env.now
                        # the value picker checks is updated last
                        self.curr_picker_allocations[picker_id] = row_id
                        rospy.loginfo("%s is allocated to %s at %0.3f" %(picker_id,
                                                                         row_id,
                                                                         self.allocation_time[row_id]))

                    # check if all rows are allocated after each allocation
                    if self.finished_allocating():
                        break

            # take a short break after each allocation loop
            yield self.env.timeout(self.process_timeout)

