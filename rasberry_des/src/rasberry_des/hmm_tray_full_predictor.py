#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import copy
import rospy
import numpy

import rasberry_des.tray_full_predictor
import rasberry_des.hmm_picker_predictor
import rasberry_des.config_utils


class HMMTrayFullPredictor(rasberry_des.tray_full_predictor.TrayFullPredictor):
    """
    """
    def __init__(self, picker_ids, env, topo_graph, n_robots, mean_idle_times, mean_tray_pick_times, mean_tray_pick_dists,
                 mean_pick_rates, mean_trans_rates, mean_unload_times, mean_load_times, forward_paths, reverse_paths, verbose=False):
        """
        """
        super(HMMTrayFullPredictor, self).__init__(picker_ids, env, topo_graph, n_robots, mean_idle_times, mean_tray_pick_dists,
                 mean_pick_rates, mean_trans_rates, mean_unload_times, mean_load_times, verbose)

        self.mean_tray_pick_times = mean_tray_pick_times

        self.fwd_state_map = forward_paths
        self.bwd_state_map = reverse_paths

        #TODO:
        # n_pick_substates might be depended on the picking rates
        # however, based on Abhishesh's prediction analysis 20 seems to be a good one with less error
        self.n_pick_substates = {picker_id:20 for picker_id in self.picker_ids}

        self.predictors = {picker_id:rasberry_des.hmm_picker_predictor.HMMPickerPredictor(picker_id, self.env,
                                                                                   self.graph, self.n_pickers, self.n_robots,
                                                                                   mean_idle_times[picker_id],
                                                                                   mean_tray_pick_times[picker_id], mean_tray_pick_dists[picker_id],
                                                                                   mean_pick_rates[picker_id], mean_trans_rates[picker_id],
                                                                                   mean_unload_times[picker_id], mean_load_times[picker_id],
                                                                                   self.fwd_state_map, self.bwd_state_map,
                                                                                   self.n_pick_substates[picker_id],
                                                                                   self.verbose) for picker_id in self.picker_ids}

    # method overloaded only for picking mode (2)
    def _update_mode_finishes(self, time_now):
        """virtually updating mode finish modes and details for all pickers based on their
        current state
        """
        # picker modes
        # 0:idle, 1:transporting to row_node, 2:picking, 3:transporting to storage,
        # 4: waiting for unload at storage, 5: waiting for loading on robot
        # 6: transporting to local storage from cold storage
        # if a picker is in the mode
        #   [0] idle, he will switch to go_to_row_node [1] after being allocated to a new row.
        #   [1] go_to_row_node, he will be starting/resuming picking [2] soon - need goal pose (node and dir)
        #   [2] picking, he will either have his tray full or pick the full row. possible next states - go_to_storage [3] (without robots) or wait_for_robot [5]
        #   [3] go_to_storage, he will be reaching storage for unloading [4]
        #   [4] wait_unloading, he will be switching to go_to_row_node [1] after unloading trays
        #   [5] wait_loading, he is waiting for robot and for loading trays on it. next states idle [0] (if curr_node = row_finish_node) or picking [2]
        #   [6] go_to_local_from_cold, he will be reaching local storage and switch to idle [0]
        for picker_id in self.picker_ids:
            msg = "%s, %s, %d" %(picker_id, self._update_needed[picker_id], self._picker_modes[picker_id])
            self.loginfo(msg)

            if not self._update_needed[picker_id]:
                # update mode only if needed
                continue

            if self._picker_modes[picker_id] == 0:
                # idle will finish in idle mode if no more rows are unallocated
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], None,
                                                        self._mode_start_times[picker_id] + self.predictors[picker_id].mean_idle_time(),
                                                        0.0]
                # if the picker had started a tray and became idle, we should send him to storage to unload partial tray
                if len(self._free_rows) == 0:
                    # if there are no unallocated rows, and not at storage, go to storage
                    storage = self.graph.local_storage_nodes[self._curr_rows[picker_id]] if self.graph.use_local_storage else self.graph.cold_storage_node
                    if self._curr_nodes[picker_id] == storage:
                        # at storage, no free rows and already idle, setting mode_finish_time
                        # as inf so that this picker will be ignored in min_event_time
                        self._mode_finish_modes[picker_id] = 0
                        self._mode_finish_details[picker_id][2] = float("inf")
                    else:
                        self._mode_finish_modes[picker_id] = 3
                else:
                    self._mode_finish_modes[picker_id] = 1
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 1:
                # started this mode earlier. will be picking after this
                # could be to start node of a row or to resume picking from an
                # intermediate row node
                self._mode_finish_details[picker_id] = [self._goal_nodes[picker_id], "forward",
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self._goal_nodes[picker_id]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 2
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 2:
                # two possible events - tray_full -> [3/5] and row_finish -> [0/3]
                predicted_tray_pick_time = self.predictors[picker_id].predict_tray_full_time() 
                remain_tray_pick_time = predicted_tray_pick_time - self._times_picked[picker_id]
                if remain_tray_pick_time > 0 and not rasberry_des.config_utils.isclose(remain_tray_pick_time, 0.):
                    print 'remain_tray_pick_time', remain_tray_pick_time
                    # remain_tray_pick_time is +ve
                    next_event, event_details = self.predictors[picker_id].predict_picking_finish_event(self._curr_nodes[picker_id],
                                                                                                        self._curr_dirs[picker_id],
                                                                                                        remain_tray_pick_time,
                                                                                                        time_now)

                    self._mode_finish_details[picker_id] = event_details # full details
                    if next_event == "tray_full":
                        self._mode_finish_modes[picker_id] = 3 if self.n_robots==0 else 5
                        self._times_picked[picker_id] = self._mode_finish_details[picker_id][2]
                        self._dists_picked[picker_id] = self._mode_finish_details[picker_id][3]
                        self._update_needed[picker_id] = False
                    elif next_event == "row_finish":
                        self._mode_finish_modes[picker_id] = 0
                        self._times_picked[picker_id] = self._mode_finish_details[picker_id][2]
                        self._dists_picked[picker_id] = self._mode_finish_details[picker_id][3]
                        self._update_needed[picker_id] = False

                else:
                    # remain_tray_pick_time is -ve, "tray_full"
                    self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], self._curr_dirs[picker_id],
                                                            time_now, self._dists_picked[picker_id]]
                    self._mode_finish_modes[picker_id] = 3 if self.n_robots==0 else 5
                    self._times_picked[picker_id] = self._mode_finish_details[picker_id][2]
                    self._dists_picked[picker_id] = self._mode_finish_details[picker_id][3]
                    self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 3:
                # next mode - unload_at_storage [4]
                # dist from curr_node to storage / trans_rate
                # storage node is already set as goal_node
                self._mode_finish_details[picker_id] = [self._goal_nodes[picker_id], None,
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self._goal_nodes[picker_id]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 4
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 4:
                # next mode - idle [0], go_to_row_node [1] or go_to_local_from_cold [6]
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], None,
                                                        time_now + self.predictors[picker_id].mean_unload_time(),
                                                        0.0]
                self._dists_picked[picker_id] = 0.0

                if len(self._free_rows) == 0:
                    self._mode_finish_modes[picker_id] = 0
                elif self.graph.use_local_storage:
                    # unloading at local -> idle
                    self._mode_finish_modes[picker_id] = 0
                else:
                    # unloading at cold -> go_to_local_from_cold
                    self._mode_finish_modes[picker_id] = 6
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 5:
                # next mode - idle [0] or picking [2]
                # if row is finished and no rows left for allocation -> idle [0]
                # else continue picking
                if self._curr_rows[picker_id] in self.graph.half_rows:
                    row_finish_node = self.graph.row_info[self._curr_rows[picker_id]][2]
                else:
                    row_finish_node = self.graph.row_info[self._curr_rows[picker_id]][1]
                self._mode_finish_details[picker_id] = [self._curr_nodes[picker_id], self._curr_dirs[picker_id],
                                                        time_now + self.predictors[picker_id].mean_unload_time(),
                                                        0.0]
                self._dists_picked[picker_id] = 0.0

                if len(self._free_rows) == 0 and self._curr_nodes[picker_id] == row_finish_node:
                    self._mode_finish_modes[picker_id] = 0
                else:
                    self._mode_finish_modes[picker_id] = 2
                self._update_needed[picker_id] = False

            elif self._picker_modes[picker_id] == 6:
                # next mode -- idle[0] at local storage
                self._mode_finish_details[picker_id] = [self.graph.local_storage_nodes[self._curr_rows[picker_id]], None,
                                                        time_now + self.predictors[picker_id].get_trans_time_to_node(self._curr_nodes[picker_id], self.graph.local_storage_nodes[self._curr_rows[picker_id]]),
                                                        0.0]
                self._mode_finish_modes[picker_id] = 0
                self._update_needed[picker_id] = False
