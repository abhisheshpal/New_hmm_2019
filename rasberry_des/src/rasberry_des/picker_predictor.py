#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import numpy


class PickerPredictor(object):
    """A class to predict a picker's actions and positions"""
    def __init__(self, picker_id, env, topo_graph, n_pickers, n_robots, verbose=False):
        """Initialiser for PickerPredictor"""
        self.picker_id = picker_id

        self.env = env
        self.graph = topo_graph
        self.n_pickers = n_pickers
        self.n_robots = n_robots
        self.verbose = verbose

        self.curr_row = None
        self.curr_node = None
        self.curr_dir = None

        # picker modes
        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        # 6: transporting to local storage from cold storage
        self.mode = []                  # list of modes
        self.mode_start_time = []       # time at which this mode is started
        self.mode_stop_time = []        # time at which this mode is finished
        self.mode_start_pose = []       # (node_id, direction) at which the mode started
        self.mode_stop_pose = []        # (node_id, direction) at which the mode stopped

        self.mode_index_tray_start = [] # index of the mode at which a tray picking is started
        self.mode_index_tray_stop = []  # index of the mode at which a tray picking is finished
        self.picking_dist_per_tray = [] # distance travelled for filling a tray
        self.picking_time_per_tray = [] # time taken to fill a tray (only picking mode)

        self.picker_modes = [0, 1, 2, 3, 4, 5, 6]
        self.n_picker_modes = len(self.picker_modes)
        self.mode_transition = numpy.zeros((self.n_picker_modes, self.n_picker_modes))
        self.time_spent_in_mode = {mode:[] for mode in self.picker_modes}

        # TODO: Pickers are assumed to start at the LS of first row in idle mode
        self.set_initial_mode_and_pose(0, self.graph.local_storage_nodes[self.graph.row_ids[0]])

    def set_initial_mode_and_pose(self, mode, node, direction=None):
        """set the initial mode and pose of the picker

        Keyword arguments:

        mode - current mode of the picker
        node - current node of the picker
        direction - picking direction of the picker
        """
        self.mode = mode
        time_now = self.env.now
        self.mode_start_time.append(time_now)
        self.mode_start_pose.append((node, direction))

    def update_mode_and_pose(self, mode, node, direction=None):
        """update the current mode of the picker

        Keyword arguments:

        mode - current mode of the picker
        node - current node of the picker
        direction - picking direction of the picker
        """
#==============================================================================
#         # update current pose
#==============================================================================
        if self.curr_node != node:
            self.curr_node = node
        if self.curr_dir != direction:
            self.curr_dir = direction

#==============================================================================
#         # is there a mode change?
#==============================================================================
        if mode != self.mode[-1]:
            # mode is changed
            time_now = self.env.now
            # close out previous mode related things
            self.mode_stop_time.append(time_now)
            self.mode_stop_pose.append((node, direction))

            # new mode related things
            self.mode.append(mode)
            self.mode_start_time.append(time_now)
            self.mode_start_pose.append((node, direction))

#==============================================================================
#             # is this a new tray event?
#==============================================================================
            # assuming picker has only one tray
            # common: curr_mode = picking
            if self.mode[-1] == 2:
                # first 0 -> 1 -> 2 is a tray start event
                if len(self.mode) == 3:
                    self.mode_index_tray_start.append(len(self.mode) - 1)

                # second tray onwards
                elif len(self.mode) > 3:
                    # without robots
                    if self.n_robots == 0:
                        if ((self.mode[-4] == 3) and (self.mode[-3] == 4) and
                            (self.mode[-2] == 1)):
                            # tray full happened in between a row
                            # resuming picking after trays are unloaded at storage
                            self.mode_index_tray_start.append(len(self.mode) - 1)
                        elif ((self.mode[-6] == 3) and (self.mode[-5] == 4) and
                              (self.mode[-4] == 6) and (self.mode[-3] == 0) and
                              (self.mode[-2] == 1)):
                            # tray full happened at the end of previous row
                            # trays unloaded at cs -> go_to_ls -> idle -> go_to_rn -> picking
                            self.mode_index_tray_start.append(len(self.mode) - 1)
                        elif ((self.mode[-5] == 3) and (self.mode[-4] == 4) and
                              (self.mode[-3] == 0) and (self.mode[-2] == 1)):
                            # tray full happened at the end of previous row
                            # trays unloaded at ls -> idle -> go_to_rn -> picking
                            self.mode_index_tray_start.append(len(self.mode) - 1)

                    # with robots
                    else:
                        if self.mode[-2] == 5:
                            # tray full happened in between a row
                            # resuming picking after trays are loaded on robot
                            self.mode_index_tray_start.append(len(self.mode) - 1)
                        elif ((self.mode[-4] == 5) and (self.mode[-3] == 0) and
                              (self.mode[-2] == 1)):
                            # tray full happened at the end of previous row
                            # trays loaded on robot -> idle -> go_to_rn -> picking
                            self.mode_index_tray_start.append(len(self.mode) - 1)

#==============================================================================
#             # is this a tray full event?
#==============================================================================
            # with robots: wait_load_robot
            # without robots: go_to_storage
            if self.mode[-1] == 3 or self.mode[-1] == 5:
                # tray_full happened at the end of previous mode
                self.mode_index_tray_stop.append(len(self.mode) -2)

                # update distance and time for this tray
                picking_time = picking_dist = 0.0
                if self.mode_index_tray_stop[-1] == self.mode_index_tray_start[-1]:
                    # no intermediate mode changes
                    idx = self.mode_index_tray_start[-1]
                    picking_time, picking_dist = self.get_picking_time_and_distance(self.mode_start_time[idx],
                                                                                    self.mode_stop_time[idx],
                                                                                    self.mode_start_pose[idx],
                                                                                    self.mode_stop_pose[idx])

                else:
                    # some mode changes in between - one or more row changes
                    # loop through the modes
                    # add distance and time, while in picking mode similar to above
                    for idx in range(self.mode_index_tray_start[-1], self.mode_index_tray_stop[-1] + 1):
                        if self.mode[idx] == 2:
                            # if in picking mode
                            _time, _dist = self.get_picking_time_and_distance(self.mode_start_time[idx],
                                                                              self.mode_stop_time[idx],
                                                                              self.mode_start_pose[idx],
                                                                              self.mode_stop_pose[idx])
                            picking_time += _time
                            picking_dist += _dist

                self.picking_time_per_tray.append(picking_time)
                self.picking_dist_per_tray.append(picking_dist)

#==============================================================================
#             # is this a new allocation?
#==============================================================================
            if (self.mode[-2] == 1) and (self.mode[-1] == 2):
                # reached a row_node and started picking
                row_id = self.graph.get_row_id_of_row_node(self.curr_node)
                if (self.curr_row is None) or (self.curr_row != row_id):
                    # curr_row will be None if not at a row_node
                    self.curr_row = row_id

#==============================================================================
#             # mode transitions and time spent in each mode
#==============================================================================
            self.mode_transition[self.mode[-2]][self.mode[-1]] += 1
            self.time_spent_in_mode[self.mode[-2]].append(self.mode_stop_time[-1] - self.mode_start_time[-2])

    def get_picking_time_and_distance(self, start_time, stop_time, start_pose, stop_pose):
        """given pose and time at both start and stop, and row_id, return the time and distance
        spent in this mode. assumes the mode is picking mode

        Keyword arguments:

        start_time - time at which mode started
        stop_time - time at which mode finished
        start_pose - pose at the start of mode
        stop_pose - pose at the finish of mode
        """
        row_id = self.graph.get_row_id_of_row_node(start_pose[0])

        assert row_id == self.graph.get_row_id_of_row_node(stop_pose[0])

        picking_time = picking_dist = 0.0

        # time
        picking_time += stop_time - start_time

        # row_id != None as the start/stop node will be a row_node
        if row_id in self.graph.half_rows:
            # row_end and dir_change nodes are the same
            row_end_node = self.graph.row_info[row_id][2]
            dir_change_node = self.graph.row_info[row_id][2]
        else:
            row_end_node = self.graph.row_info[row_id][1]
            dir_change_node = self.graph.row_info[row_id][2]

        # distance
        # is there a direction change?
        if stop_pose[1] is None:
            # stop_pose (may be with tray_full) at end_of_row
            # could be the start_row_node or last_row_node (depending on half/full row)
            if start_pose[1] == "reverse":
                # row_end_node is row_start_node
                picking_dist += self.graph.get_path_details(start_pose[0], row_end_node)[2]
            else:
                # start_pose[1] = "forward"
                dist_forward = dist_reverse = 0.0
                if row_id in self.graph.half_rows:
                    dist_forward = self.graph.get_path_details(start_pose[0], row_end_node)[2]
                else:
                    dist_forward = self.graph.get_path_details(start_pose[0], row_end_node)[2]
                    dist_reverse = self.graph.get_path_details(dir_change_node, row_end_node)[2]
                picking_dist += dist_forward + dist_reverse

        elif stop_pose[1] == start_pose[1]:
            # tray_full not at end_of_row
            picking_dist += self.graph.get_path_details(start_pose[0], stop_pose[0])[2]

        return (picking_time, picking_dist)

#    def predict_current_tray_full(self, ):
#        """predict when the current tray could be full"""
#        # tray full -> mode 3 (without robots) or mode 5 (with robots)
#        # predict only if a new tray has been initialised, else return None
#        if len(self.mode_index_tray_start) <= len(self.mode_index_tray_stop):
#            return None
#
#        # No prediction without prior data
#        if len(self.picking_time_per_tray) == 0:
#            return None
#
#        # factors to be considered for prediction
#        # 1. what is the average picking speed / tray
#        # 2. details about the current row
#        # 3. which could be next row?
#        # 4. what is the average transportation speed
#        # 5. what is the average yield per unit distance?
#        # 6. which will be the first - row_finish or tray_full (should consider yield as well)?
#        # 7. what are the average time spend in the other intermediate modes
#
#        #
#
#        if self.n_robots == 0:
#            # from the current state what are the transition probabilities to mode 3
#            # if in picking, what are the probable tray full times in future?
#            # if not in picking, is a tray is partially filled, when could it be full?
#            pass
#        else:
#            # from the current state what are the transition probabilities to mode 5
#            # if in picking, what are the probable tray full times in future?
#            # if not in picking, is a tray is partially filled, when could it be full?
#            pass
#
#
#
#
#
#
#
#
#        # remaining time is calculated only if not in picking mode now
#        if self.mode[-1] != 2:
#
#            pass




