#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY
@author: abhisheshpal
@author: gpdas
"""
import numpy

import rasberry_des.hmmodel
import rasberry_des.picker_predictor
import rasberry_des.hmm_picker_predictor
import rasberry_des.config_utils


class HMMPickerPredictor(rasberry_des.picker_predictor.PickerPredictor):
    """
    """
    def __init__(self, picker_id, env, topo_graph, n_pickers, n_robots, mean_idle_time, mean_tray_pick_time, mean_tray_pick_dist,
                 mean_pick_rate, mean_trans_rate, mean_unload_time, mean_load_time, fwd_state_map, bwd_state_map, n_pick_substates,
                 verbose=False):
        """
        """
        super(HMMPickerPredictor, self).__init__(picker_id, env, topo_graph, n_pickers, n_robots, mean_idle_time, mean_tray_pick_dist,
                 mean_pick_rate, mean_trans_rate, mean_unload_time, mean_load_time, verbose)

        self.fwd_state_map = fwd_state_map
        self.bwd_state_map = bwd_state_map
        self.n_pick_substates = n_pick_substates
        self.mean_node_dist = numpy.mean(self.graph.mean_node_dist.values())

        self.mean_tray_pick_time = lambda: numpy.mean(self.picking_times_per_tray) if numpy.size(self.picking_times_per_tray) != 0. else mean_tray_pick_time
        self.predict_interval = 120 # time period to which the prediction is made in a loop. This must be < node-to-node picking time

        self._init_hmmodels()


    def _init_hmmodels(self, ):
        """
        """
#==============================================================================
#         # pick_substates_model
#==============================================================================
        # optimum number of substates depends on the picking rate of the picker
        rate_pick_substates = (1.0 / self.mean_tray_pick_time()) * self.n_pick_substates
        
        
        
        # defining a state map to change through the substates. 0 --> 1 --> 2 --> 3 --> 4 --> 5 --> 0
        state_map_pick_substates = numpy.eye(self.n_pick_substates, k=1) # connect all the successive nodes
        state_map_pick_substates[-1,0] = 1
        state_map_pick_substates += numpy.eye(self.n_pick_substates, k=-1) * 0.1 # A small probability to go back, connect all the successive nodes backward
        state_map_pick_substates[0,-1] = 0.1

        rs_pick_substates = numpy.sum(state_map_pick_substates, axis=1)

        # transition rate matrix
        Q_pick_substates = (numpy.diag(-rs_pick_substates) + state_map_pick_substates) * rate_pick_substates

        # creating observation matrix, assuming each states has ~70% prob to emit the state itself as observation
        # and another ~10% for neighbouring states each (confusing them). and +.1% for all observations
        # for numerical stability
        B_pre_pick_substates = numpy.ones(self.n_pick_substates) * .001 + numpy.eye(self.n_pick_substates) * .7 + numpy.eye(self.n_pick_substates, k=1) * .1 + numpy.eye(self.n_pick_substates, k=-1) * .1
        # adding 10% change of "unknown" observation which each state is equally likely to emit (used for prediction)
        B_pick_substates = numpy.transpose(numpy.vstack([B_pre_pick_substates, [.101] * self.n_pick_substates]))  # numpy.vstack will add extra column in B_pre matrix vertically
        B_pick_substates[0,-2] = .101     # first row and second last column is filled with 0.101
        B_pick_substates[-1,0] = .101     # Last row and first columm is filled with 0.101
        # normalise B (make sure probs sum up to 1)
        row_sums = B_pick_substates.sum(axis=1)
        B_pick_substates = B_pick_substates / row_sums[:, numpy.newaxis]
        # Pi is the vector of initial state probabilities. Assuming uniform here
        # (We may make a stronger assumption here at some point)
        Pi_pick_substates = numpy.array([1.0 / self.n_pick_substates] * self.n_pick_substates )   # Uniform Pi for all substates
        self.pick_substates_model = rasberry_des.hmmodel.HMModel(self.n_pick_substates,
                                                                  from_file=False,
                                                                  trans_rate_mat=Q_pick_substates,
                                                                  obs_prob_mat=B_pick_substates,
                                                                  init_state_prob=Pi_pick_substates
                                                                  )

        # TODO:
        # forward and backward models may not be necessary. It could be assumed that the picker is moving at a constant rate and decide where he will be
        # as is done in tray_full_predictor

#==============================================================================
#         # forward picking model
#==============================================================================
        # # summing all column of adjency matrix
        # rs_fwd_pick = numpy.sum(self.fwd_state_map, axis=1)       # it sums up all the columns of a single row,so that it can help in defining Q in next step
        # # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
        # # expected mean rate in per seconds
        # rate_fwd_pick =   self.mean_pick_rate() / self.mean_node_dist # The picking_rate (0.001724078) is calculated from DES
        # Q_fwd_pick = (numpy.diag(-rs_fwd_pick) + self.fwd_state_map) * rate_fwd_pick # Keep in mind that, sum(Qij) = -Qii =< 1.
        

        # B_pre_fwd_pick = numpy.ones(self.fwd_state_map.size) * .001 + numpy.eye(self.fwd_state_map.size) * .7 + numpy.eye(self.fwd_state_map.size, k=1) * .02 + numpy.eye(self.fwd_state_map.size, k=-1) * .02 + numpy.eye(self.fwd_state_map.size, k=2) * .01 + numpy.eye(self.fwd_state_map.size, k=-2) * .01
        # B_fwd_pick = numpy.transpose(numpy.vstack([B_pre_fwd_pick, [.101] * self.fwd_state_map.size]))  # np.vstack will add extra column in B_pre matrix vertically
        # B_fwd_pick[0,-2] = .101     # first row and second last column is filled with 0.101
        # B_fwd_pick[-1,0] = .101     # Last row and first columm is filled with 0.101
        # row_sums = B_fwd_pick.sum(axis=1)
        # B_fwd_pick = B_fwd_pick / row_sums[:, numpy.newaxis]
        # B_fwd_pick = [] # observation probability matrix

        # Pi_fwd_pick = numpy.array([1.0 / self.fwd_state_map.size] * self.fwd_state_map.size )   # Uniform Pi for all substates

        # self.fwd_picking_model = rasberry_des.hmmodel.HMModel(self.fwd_state_map.size,
        #                                                       from_file=False,
        #                                                       trans_rate_mat=Q_fwd_pick,
        #                                                       obs_prob_mat=B_fwd_pick,
        #                                                       init_state_prob=Pi_fwd_pick
        #                                                       )
        
        
        ## changed B_pre_fwd_pick shape to solve issues of memory ---
   
        # summing all column of adjency matrix
        rs_fwd_pick = numpy.sum(self.fwd_state_map, axis=1)       # it sums up all the columns of a single row,so that it can help in defining Q in next step
        # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
        # expected mean rate in per seconds
        rate_fwd_pick =   self.mean_pick_rate() / self.mean_node_dist # The picking_rate (0.001724078) is calculated from DES
        Q_fwd_pick = (numpy.diag(-rs_fwd_pick) + self.fwd_state_map) * rate_fwd_pick # Keep in mind that, sum(Qij) = -Qii =< 1.
        f = len(self.fwd_state_map[:, 0]) # self.fwd_state_map[:, 0]
        B_pre_fwd_pick = numpy.ones(f) * .001 + numpy.eye(f) * .7 + numpy.eye(f, k=1) * .02 + numpy.eye(f, k=-1) * .02 + numpy.eye(f, k=2) * .01 + numpy.eye(f, k=-2) * .01
        B_fwd_pick = numpy.transpose(numpy.vstack([B_pre_fwd_pick, [.101] * f]))  # np.vstack will add extra column in B_pre matrix vertically
        B_fwd_pick[0,-2] = .101     # first row and second last column is filled with 0.101
        B_fwd_pick[-1,0] = .101     # Last row and first columm is filled with 0.101
        row_sums = B_fwd_pick.sum(axis=1)
        B_fwd_pick = B_fwd_pick / row_sums[:, numpy.newaxis]
        B_fwd_pick = [] # observation probability matrix

        Pi_fwd_pick = numpy.array([1.0 / f] * f )   # Uniform Pi for all substates
        self.fwd_picking_model = rasberry_des.hmmodel.HMModel(f,
                                                              from_file=False,
                                                              trans_rate_mat=Q_fwd_pick,
                                                              obs_prob_mat=B_fwd_pick,
                                                              init_state_prob=Pi_fwd_pick
                                                              )
                
     
#==============================================================================
#         # backward picking models
#==============================================================================
        # # summing all column of adjency matrix
        # rs_bwd_pick = numpy.sum(self.bwd_state_map, axis=1)       # it sums up all the columns of a single row,so that it can help in defining Q in next step
        # # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
        # # expected mean rate in per seconds
        # rate_bwd_pick =   self.mean_pick_rate() / self.mean_node_dist # The picking_rate (0.001724078) is calculated from DES
        # Q_bwd_pick = (numpy.diag(-rs_bwd_pick) + self.bwd_state_map) * rate_bwd_pick # Keep in mind that, sum(Qij) = -Qii =< 1.

        # back = len(self.bwd_state_map[:,0])
        # print back

        # B_pre_bwd_pick = numpy.ones(self.bwd_state_map.size) * .001 + numpy.eye(self.bwd_state_map.size) * .7 + numpy.eye(self.bwd_state_map.size, k=1) * .02 + numpy.eye(self.bwd_state_map.size, k=-1) * .02 + numpy.eye(self.bwd_state_map.size, k=2) * .01 + numpy.eye(self.bwd_state_map.size, k=-2) * .01
        # B_bwd_pick = numpy.transpose(numpy.vstack([B_pre_bwd_pick, [.101] * self.bwd_state_map.size]))  # np.vstack will add extra column in B_pre matrix vertically
        # B_bwd_pick[0,-2] = .101     # first row and second last column is filled with 0.101
        # B_bwd_pick[-1,0] = .101     # Last row and first columm is filled with 0.101
        # # normalise B (make sure probs sum up to 1)
        # row_sums = B_bwd_pick.sum(axis=1)
        # B_bwd_pick = B_bwd_pick / row_sums[:, numpy.newaxis]
        # B_bwd_pick = [] # observation probability matrix

        # Pi_bwd_pick = numpy.array([1.0 / self.bwd_state_map.size] * self.bwd_state_map.size )   # Uniform Pi for all substates

        # self.bwd_picking_model = rasberry_des.hmmodel.HMModel(self.bwd_state_map.size,
        #                                                       from_file=False,
        #                                                       trans_rate_mat=Q_bwd_pick,
        #                                                       obs_prob_mat=B_bwd_pick,
        #                                                       init_state_prob=Pi_bwd_pick
        #                                                       )
        
        
        ## changed B_pre_bwd_pick shape to solve issues of memory ---
        
        # summing all column of adjency matrix
        rs_bwd_pick = numpy.sum(self.bwd_state_map, axis=1)       # it sums up all the columns of a single row,so that it can help in defining Q in next step
        # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
        # expected mean rate in per seconds
        rate_bwd_pick =   self.mean_pick_rate() / self.mean_node_dist # The picking_rate (0.001724078) is calculated from DES
        Q_bwd_pick = (numpy.diag(-rs_bwd_pick) + self.bwd_state_map) * rate_bwd_pick # Keep in mind that, sum(Qij) = -Qii =< 1.
        back = len(self.bwd_state_map[:,0])
        B_pre_bwd_pick = numpy.ones(back) * .001 + numpy.eye(back) * .7 + numpy.eye(back, k=1) * .02 + numpy.eye(back, k=-1) * .02 + numpy.eye(back, k=2) * .01 + numpy.eye(back, k=-2) * .01
        B_bwd_pick = numpy.transpose(numpy.vstack([B_pre_bwd_pick, [.101] * back]))  # np.vstack will add extra column in B_pre matrix vertically
        B_bwd_pick[0,-2] = .101     # first row and second last column is filled with 0.101
        B_bwd_pick[-1,0] = .101     # Last row and first columm is filled with 0.101
        # normalise B (make sure probs sum up to 1)
        row_sums = B_bwd_pick.sum(axis=1)
        B_bwd_pick = B_bwd_pick / row_sums[:, numpy.newaxis]
        B_bwd_pick = [] # observation probability matrix
        Pi_bwd_pick = numpy.array([1.0 / back] * back )   # Uniform Pi for all substates
        self.bwd_picking_model = rasberry_des.hmmodel.HMModel(back,
                                                              from_file=False,
                                                              trans_rate_mat=Q_bwd_pick,
                                                              obs_prob_mat=B_bwd_pick,
                                                              init_state_prob=Pi_bwd_pick
                                                              )
        
        

    def predict_tray_full_time(self, ):
        """predict when the tray will be full for the picker
        """
        # TODO: take the progress (completed substates) into consideration
        # now prediction is assuming picking mode started at time zero
        n_iter = 1.25 * (self.mean_tray_pick_time() / self.predict_interval) # 25 % extra time to check progress
        # print int(numpy.floor(n_iter))
        obs = [0, 0]
        curr_substate = 0
        prev_substate = 0
        tray_pick_time = 0.
        
        # needed type int to iterate over range
        for i in range(int(n_iter)):
            (state, kl, posterior) = self.fwd_picking_model.predict(obs,
                                                                    predict_time = i,
                                                                    verbose = False
                                                                    )
            tray_pick_time += self.predict_interval

            if state== 0 and (prev_substate == self.n_pick_substates - 2 or curr_substate == self.n_pick_substates - 1):
                break

            prev_substate = curr_substate
            curr_substate = state
            obs = [state, state]
        return tray_pick_time

    def predict_picking_finish_event(self, curr_node, curr_dir, remain_tray_pick_time, dist_so_far, time_now):
        """predict which of tray_full and row_finish will be the first event
        if tray_full, event: `tray_full`, details: [node, dir, time, dist_so_far]
        if row_finish, event: `row_finish`, details: [node, dir, time, dist_so_far]

        this method works only when the picker is in the picking mode

        Keyword arguments:
        curr_node -- current node
        curr_dir -- current direction
        remain_tray_pick_time -- remaining picking time as per prediction
        time_now -- current time
        """
        next_event = None
        event_details = []

        try:
            assert remain_tray_pick_time > 0. and not rasberry_des.config_utils.isclose(remain_tray_pick_time, 0.)
        except:
            # remaining dist is less than zero
            next_event = "tray_full"
            event_details.append(curr_node)
            event_details.append(curr_dir)
            event_details.append(time_now)
            event_details.append(dist_so_far + 0.0)
            return (next_event, event_details)

        # will it be finished in the current row?
        # if picking would not be finishing in this row, only node returnd is the row_end_node
        # if picking could be finishing this row, there will be two nodes (just before and after the
        # distance travelled equals the target distance) returned
        cn_nodes, cn_dists, cn_dirs, cn_times = self.get_closest_event_nodes_while_picking(curr_node, curr_dir, remain_tray_pick_time)

        if len(cn_times) == 2:
            # will finish in this row - two nodes in cn_nodes
            # now return the closer one
            # TODO: better to return both with some probability?
            next_event = "tray_full"
            if abs(remain_tray_pick_time - cn_times[0]) < abs(remain_tray_pick_time - cn_times[1]):
                # picking_to_finish, remain_dist is cn_dists[0]
                event_details.append(cn_nodes[0])
                event_details.append(cn_dirs[0])
                event_details.append(time_now + cn_times[0]) # picking time
                event_details.append(dist_so_far + cn_dists[0])
            else:
                # picking_to_finish, remain_dist is cn_dists[1]
                event_details.append(cn_nodes[1])
                event_details.append(cn_dirs[1])
                event_details.append(time_now + cn_times[1]) # picking time
                event_details.append(dist_so_far + cn_dists[1])

        else:
            # won't finish in this row - one node and distance returned
            next_event = "row_finish"
            event_details.append(cn_nodes[0])
            event_details.append(cn_dirs[0])
            event_details.append(time_now + cn_times[0]) # picking time
            event_details.append(dist_so_far + cn_dists[0])

        return (next_event, event_details)

    def get_closest_event_nodes_while_picking(self, curr_node, curr_dir, remain_tray_pick_time):
        """given a row_id and a distance, return the closest nodes while picking
        along with the remaining distances and directions at these nodes.

        if picking would not be finishing in this row, only node returnd is the row_end_node
        if picking could be finishing this row, there will be two nodes (just before and after the
        distance travelled equals the target distance) returned

        keyword arguments:

        curr_node -- current node
        curr_dir -- current picking direction
        remain_tray_pick_time -- remaining picking time to fill the current tray
        """
        row_id = self.graph.get_row_id_of_row_node(curr_node)

        try:
            assert remain_tray_pick_time > 0. and not rasberry_des.config_utils.isclose(remain_tray_pick_time, 0.)
        except:
            msg = "remain_tray_pick_time: %0.1f" %(remain_tray_pick_time)
            raise Exception(msg)

        _remain_tray_pick_time = remain_tray_pick_time

        closest_nodes = [] # nodes where the picker could endup. 2 nodes for possible tray_full. 1 node for row_finish
        finish_dists = []
        finish_dirs = []
        finish_times = [] # actual time picked in this method

        dist_so_far = 0.0

        # predict in a loop until the current row is finished or up to the remaining_pick_time whichever comes first
        row_change = False
        dir_change = False
        prev_node = curr_node
        prev_dir = curr_dir
        n_iter = _remain_tray_pick_time / self.predict_interval
        n_nodes = 0.
        for _row_id in self.graph.row_ids:
            if _row_id == row_id:
                break
            n_nodes += len(self.graph.row_nodes[_row_id])

        for i in range(n_iter):
            curr_node_idx = n_nodes + self.graph.row_nodes[row_id].index(curr_node)
            obs = [curr_node_idx, curr_node_idx]
            if curr_dir == "forward":
                (state, kl, posterior) = self.fwd_picking_model.predict(obs,
                                                                        predict_time = i,
                                                                        verbose = False
                                                                        )
            elif curr_dir == "reverse":
                (state, kl, posterior) = self.bwd_picking_model.predict(obs,
                                                                        predict_time = i,
                                                                        verbose = False
                                                                        )
            else:
                raise Exception("wrong picking direction")

            print state
            # state is an index in the state map containing all rows
            # row can change when the state > max index for that row or
            # when a jump happens from last to first row
            if ((state >= n_nodes + len(self.graph.row_nodes[row_id])) or
                ((n_nodes > 0) and (state < len(self.graph.row_nodes[self.graph.row_ids[0]])))):
                # row_id of the predicted node has changed => either row or dir is changed
                if curr_dir == "forward" and row_id not in self.graph.half_rows:
                    curr_dir = "reverse"
                    dir_change = True
                elif curr_dir == "forward" and row_id in self.graph.half_rows:
                    row_change = True
                elif curr_dir == "reverse":
                    row_change = True
                break

            _remain_tray_pick_time -= self.predict_interval
            prev_node = curr_node
            curr_node = self.graph.row_nodes[row_id][state - n_nodes]
            if prev_node != curr_node:
                _, _, route_dists = self.graph.get_path_details(prev_node, curr_node)
                dist_so_far += sum(route_dists)

        if dir_change:
            # picking in reverse dir now
            # predict until row is completed or tray is full
            n_iter = _remain_tray_pick_time / self.predict_interval
            for i in range(n_iter):
                curr_node_idx = n_nodes + self.graph.row_nodes[row_id].index(curr_node)
                obs = [curr_node_idx, curr_node_idx]

                (state, kl, posterior) = self.bwd_picking_model.predict(obs,
                                                                        predict_time = i,
                                                                        verbose = False
                                                                        )
                # state is an index in the state map containing all rows
                # row can change when the state > max index for that row or
                # when a jump happens from last to first row
                if ((state >= n_nodes + len(self.graph.row_nodes[row_id])) or
                    ((n_nodes > 0) and (state < len(self.graph.row_nodes[self.graph.row_ids[0]])))):
                    # row_id of the predicted node has changed => row is changed
                    row_change = True
                    break

                _remain_tray_pick_time -= self.predict_interval
                prev_node = curr_node
                curr_node = self.graph.row_nodes[row_id][state - n_nodes]
                prev_dir = curr_dir
                if prev_node != curr_node:
                    _, _, route_dists = self.graph.get_path_details(prev_node, curr_node)
                    dist_so_far += sum(route_dists)

        if row_change:
            # no more predictions needed. at the end_nde node for half_row or at start_node for full row
            closest_nodes.append(curr_node)
            finish_dists.append(dist_so_far)
            finish_dirs.append(curr_dir)
            finish_times.append(remain_tray_pick_time - _remain_tray_pick_time)
        else:
            # tray will be full here
            closest_nodes.append(prev_node)
            finish_dists.append(dist_so_far - sum(route_dists))
            finish_dirs.append(prev_dir)
            finish_times.append(remain_tray_pick_time - _remain_tray_pick_time + self.predict_interval)

            closest_nodes.append(curr_node)
            finish_dists.append(dist_so_far)
            finish_dirs.append(curr_dir)
            finish_times.append(remain_tray_pick_time - _remain_tray_pick_time)

        return (closest_nodes, finish_dists, finish_dirs, finish_times)


