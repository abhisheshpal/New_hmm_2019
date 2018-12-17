#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import copy
import rospy

import rasberry_des.picker_predictor
import rasberry_des.config_utils


class TrayFullPredictor(object):
    """
    A class that can predict when pickers are going to finish their current tray
    """
    def __init__(self, picker_ids, env, topo_graph, n_robots, mean_idle_time, mean_tray_pick_dist,
                 mean_pick_rate, mean_trans_rate, mean_unload_time, mean_load_time, verbose=False):
        """
        initialise the TrayFullPredictor object

        Keyword arguments:

        picker_ids -- list of picker_ids
        env -- simpy.Environment object
        topo_graph -- rasberry_des.TopoGraph object
        n_robots -- number of robots
        """
        self.picker_ids = picker_ids
        self.env = env
        self.graph = topo_graph
        self.n_pickers = len(self.picker_ids)
        self.n_robots = n_robots
        self.verbose = verbose

        self._allocated_rows = []
        self._unallocated_rows = copy.deepcopy(self.graph.row_ids)
        self._free_rows = copy.deepcopy(self._unallocated_rows)

        # PickerPredictor is used as logging object for each picker
        self.predictors = {picker_id:rasberry_des.picker_predictor.PickerPredictor(picker_id, self.env,
                                                                                   self.graph, self.n_pickers, self.n_robots,
                                                                                   mean_idle_time, mean_tray_pick_dist,
                                                                                   mean_pick_rate, mean_trans_rate,
                                                                                   mean_unload_time, mean_load_time,
                                                                                   self.verbose) for picker_id in self.picker_ids}

    def set_initial_mode_and_pose(self, picker_id, mode, node, direction=None):
        """call the set_initial_mode_and_pose method of the picker with picker_id
        """
        self.predictors[picker_id].set_initial_mode_and_pose(mode, node, direction)

    def update_mode_and_pose(self, picker_id, mode, node, direction=None, goal_node=None):
        """call the update_mode_and_pose method of the picker with picker_id
        """
        self.predictors[picker_id].update_mode_and_pose(mode, node, direction, goal_node)

    def update_unallocated(self, row_id):
        """remove a row_id from unallocated when a row is actually allocated to a picker
        """
        self._unallocated_rows.remove(row_id)

    def predict_tray_full(self, ):
        """predict when and where the pickers (who are in the picking mode now) will have their
        trays full.

        TODO: current implementation assumes clear sequeces when modes are changed

        assuming mode changes happen only when the current mode is finished normally,
        e.g. picking - tray_full or row_finish; idle - new_alloc.
        when needed based on the MDP probability to switch to another mode from current,
        predict the next mode.
        """
        predictions = {}
        for picker_id in self.picker_ids:
            predictions[picker_id] = self.predictors[picker_id].predict_current_tray_full()

        return predictions

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
