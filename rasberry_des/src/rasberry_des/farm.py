#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import simpy
import rasberry_des.topo
import rospy
import geometry_msgs.msg
import rasberry_des.msg


class Farm(object):
    """Farm class definition"""
    def __init__(self, name, env, des_env, n_topo_nav_rows, topo_graph, picker_ids, robot_ids):
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

        # publishers / subscribers
        self.picker_pose_subs = {}
        self.picker_poses = {picker_id:geometry_msgs.msg.Pose() for picker_id in self.picker_ids}
        self.picker_status_subs = {}
        self.picker_statuses = {picker_id:rasberry_des.msg.Picker_Status() for picker_id in self.picker_ids}
        self.init_picker_subs()

        self.robot_pose_subs = {}
        self.robot_poses = {robot_id:geometry_msgs.msg.Pose() for robot_id in self.robot_ids}
        self.robot_status_subs = {}
        self.robot_statuses = {robot_id:rasberry_des.msg.Robot_Status() for robot_id in self.robot_ids}
        self.init_robot_subs()

        # services / clients
        self.tray_full_service = rospy.Service('tray_full', rasberry_des.srv.Trays_Full, self.update_tray_full)
        self.picker_unload_service = rospy.Service('tray_unload', rasberry_des.srv.Trays_Full, self.update_tray_unload)

        self.pickers_with_tray_full = []
        self.pickers_full_trays = {}
        self.tot_trays_unloaded = 0.0

        print des_env
        if des_env == "simpy":
            self.process_timeout = 1.0
        elif des_env == "ros":
            self.process_timeout = 0.1

    def update_tray_full(self, srv):
        self.pickers_with_tray_full.append(srv.picker_id)
        self.pickers_full_trays[srv.picker_id] = srv.n_trays
        return rasberry_des.srv.Trays_FullResponse()

    def update_tray_unload(self, srv):
        self.tot_trays_unloaded += srv.n_trays
        self.pickers_with_tray_full.remove(srv.picker_id)
        self.pickers_full_trays[srv.picker_id] = 0
        return rasberry_des.srv.Trays_FullResponse()

    def init_picker_subs(self, ):
        """initialise picker related subscribers"""
        ns = rospy.get_namespace()
        for picker_id in self.picker_ids:
            self.picker_pose_subs[picker_id] = rospy.Subscriber(ns + "%s/pose"%(picker_id),
                                                                geometry_msgs.msg.Pose,
                                                                self.update_picker_position,
                                                                callback_args=picker_id)

            self.picker_status_subs[picker_id] = rospy.Subscriber(ns + "%s/status"%(picker_id),
                                                                  rasberry_des.msg.Picker_Status,
                                                                  self.update_picker_status,
                                                                  callback_args=picker_id)

    def update_picker_position(self, msg, picker_id):
        """callback for pose topics from pickers"""
        self.picker_poses[picker_id].position.x = msg.position.x
        self.picker_poses[picker_id].position.y = msg.position.y
        self.picker_poses[picker_id].position.z = msg.position.z
        self.picker_poses[picker_id].orientation.x = msg.orientation.x
        self.picker_poses[picker_id].orientation.y = msg.orientation.y
        self.picker_poses[picker_id].orientation.z = msg.orientation.z
        self.picker_poses[picker_id].orientation.w = msg.orientation.w

    def update_picker_status(self, msg, picker_id):
        """callback for status topics from pickers"""
        if self.picker_statuses[picker_id].picker_id != picker_id:
            # update constatnts only once
            self.picker_statuses[picker_id].picker_id = msg.picker_id
            self.picker_statuses[picker_id].picking_rate = msg.picking_rate
            self.picker_statuses[picker_id].transportation_rate = msg.transportation_rate
            self.picker_statuses[picker_id].tray_capacity = msg.tray_capacity

        self.picker_statuses[picker_id].picking_progress = msg.picking_progress
        self.picker_statuses[picker_id].n_trays = msg.n_trays
        self.picker_statuses[picker_id].tot_trays = msg.tot_trays
        self.picker_statuses[picker_id].n_rows = msg.n_rows
        self.picker_statuses[picker_id].curr_row = msg.curr_row
        self.picker_statuses[picker_id].mode = msg.mode

    def init_robot_subs(self, ):
        """initialise robot related subscribers"""
        ns = rospy.get_namespace()
        for robot_id in self.robot_ids:
            self.robot_pose_subs[robot_id] = rospy.Subscriber(ns + "%s/pose"%(robot_id),
                                                              geometry_msgs.msg.Pose,
                                                              self.update_robot_position,
                                                              callback_args=robot_id)

            self.robot_status_subs[robot_id] = rospy.Subscriber(ns + "%s/status"%(robot_id),
                                                                rasberry_des.msg.Robot_Status,
                                                                self.update_robot_status,
                                                                callback_args=robot_id)

    def update_robot_position(self, msg, robot_id):
        """callback for pose topics from robots"""
        self.robot_poses[robot_id].position.x = msg.position.x
        self.robot_poses[robot_id].position.y = msg.position.y
        self.robot_poses[robot_id].position.z = msg.position.z
        self.robot_poses[robot_id].orientation.x = msg.orientation.x
        self.robot_poses[robot_id].orientation.y = msg.orientation.y
        self.robot_poses[robot_id].orientation.z = msg.orientation.z
        self.robot_poses[robot_id].orientation.w = msg.orientation.w

    def update_robot_status(self, msg, robot_id):
        """callback for status topics from robots"""
        if self.robot_statuses[robot_id].robot_id != robot_id:
            # update constatnts only once
            self.robot_statuses[robot_id].robot_id = msg.robot_id
            self.robot_statuses[robot_id].transportation_rate = msg.transportation_rate
            self.robot_statuses[robot_id].max_n_trays = msg.max_n_trays

        self.robot_statuses[robot_id].n_empty_trays = msg.n_empty_trays
        self.robot_statuses[robot_id].n_full_trays = msg.n_full_trays
        self.robot_statuses[robot_id].mode = msg.mode

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

        while True:
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
