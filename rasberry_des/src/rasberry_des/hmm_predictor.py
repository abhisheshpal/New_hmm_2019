#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rasberry_des


class HMMPredictor(object):
    """
    """
    def __init__(self, picker_ids, tray_full_times, picking_rates, transportation_rates, topo_graph, forward_paths, reverse_paths):
        """
        Keyword arguments:

        picker_ids - picker ids, list
        tray_full_times - mean time each picker will take to fill a tray, dict
        picking_rates - transition speed when picking, dict
        transportation_rates - transition speed when transporting, dict
        topo_graph - rasberry_des.topo.TopoGraph object
        forward_paths - for picking in forward direction - smaller topo state map with only forward velocities in each row
        reverse_paths - for pickign in reverse direction - smaller topo state map with only reverse velocities in each row

        For each picker,
            a CtHMM for predicting when the picking mode will be finished
            a CtHMM for predicting transition along the topological map during picking
                - can't take a route to predict when it reaches the end_node / goal_node
                - may be two CtHMM for forward / reverse picking
        """
        # create predictors for all pickers
        self.picker_ids = picker_ids
        self.tray_full_times = tray_full_times # {picker_id: mean_tray_full_time of picker}
        self.picking_rates = picking_rates # {picker_id: speed of picker while picking}
        self.transportation_rates = transportation_rates # {picker_id: speed of picker while transporting}
        self.forward_paths = forward_paths
        self.reverse_paths = reverse_paths

        self.n_picking_mode_substates = 10 # 10 percentage increments

        # track picker states
        self.current_modes = {picker_id: 0 for picker_id in self.picker_ids}
        self.current_mode_start_times = {picker_id: 0 for picker_id in self.picker_ids}
        self.current_mode_row_ids = {picker_id: None for picker_id in self.picker_ids}
        self.current_mode_node_ids = {picker_id: None for picker_id in self.picker_ids}
        self.current_mode_picking_directions = {picker_id: "forward" for picker_id in self.picker_ids}


        self.mode_models = {} # will give time at which the tray would be full
        self.forward_picking_models = {} # used to predict where a picker will be in a given time period
        self.reverse_picking_models = {} # used to predict where a picker will be in a given time period
        self.init_models()

    def get_trayfull_predictions(self, ):
        """get the state of a robot"""
        for picker_id in self.picker_ids:
            # make predictions
            # needs all picker_ids, their current states, time at which these current states started,
            print (picker_id,
                   self.current_modes[picker_id],
                   self.current_mode_start_times[picker_id],
                   self.current_mode_row_ids[picker_id],
                   self.current_mode_node_ids[picker_id],
                   self.current_mode_picking_directions[picker_id])

        # finds state, row_id, node_id, picking_dir, time for all pickers in picking mode
        predictions = {}

        #TODO:
        # Use mode_models to get a time at which each picker will finish his trays
        # use forward_picking_models to find when the picker reaches end of row / where the picker will reach in the given time during forward picking
        # use reverse_picking_models to find when the picker reaches end of row / where the picker will reach in the given time during reverse picking
        # find the sequence of events from different pickers to decide the assignment of next rows (see tray_full_predictor)

        return predictions

    def init_models(self, ):
        """
        """
        # mode_models
        B = [] # observation probability matrix
        Pi = [] # initial state probabilities, np.array([1.0 / n_states] * n_states )
        for picker_id in self.picker_ids:
            Q = [] # state transition matrix, depends on the rates of the picker
            self.mode_models[picker_id] = rasberry_des.hmmodel.HMModel(self.n_picking_substates,
                                                                       from_file=False,
                                                                       trans_rate_mat=Q,
                                                                       obs_prob_mat=B,
                                                                       init_state_prob=Pi
                                                                       )
        # TODO:
        # forward and reverse models may not be necessary. It could be assumed that the picker is moving at a constant rate and decide where he will be
        # as is done in tray_full_predictor

        # forward picking models
        B = [] # observation probability matrix
        Pi = [] # initial state probabilities, np.array([1.0 / n_states] * n_states )
        for picker_id in self.picker_ids:
            Q = [] # state transition matrix, depends on the rates of the picker
            self.forward_picking_models[picker_id] = rasberry_des.hmmodel.HMModel(self.n_picking_substates,
                                                                                  from_file=False,
                                                                                  trans_rate_mat=Q,
                                                                                  obs_prob_mat=B,
                                                                                  init_state_prob=Pi
                                                                                  )
        # reverse picking models
        B = [] # observation probability matrix
        Pi = [] # initial state probabilities, np.array([1.0 / n_states] * n_states )
        for picker_id in self.picker_ids:
            Q = [] # state transition matrix, depends on the rates of the picker
            self.reverse_picking_models[picker_id] = rasberry_des.hmmodel.HMModel(self.n_picking_substates,
                                                                                  from_file=False,
                                                                                  trans_rate_mat=Q,
                                                                                  obs_prob_mat=B,
                                                                                  init_state_prob=Pi
                                                                                  )
