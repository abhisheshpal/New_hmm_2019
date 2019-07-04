#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@author: marc-hanheide
@author: abhisheshpal
@author: gpdas
"""

"""
Sequence of hmm models in this script
- FORWARD PICKING prediction model of MODE 2 within SUBSTATEs
- FORWARD PICKING : mode2_node_model
- BACKWARD PICKING : mode2_node_model

"""

#==============================================================================
#
#==============================================================================
import numpy as np
import sys
import operator
import os

from rasberry_des.hmmodel import HMModel


np.set_printoptions(threshold=sys.maxsize)


if __name__ == "__main__":

#==============================================================================
# prediction model of MODE 2 within SUBSTATEs
#==============================================================================

    print ('\n PICKING prediction model of MODE 2 within SUBSTATEs: mode2_model \n')
    # optimum number of substates depends on the picking rate of the picker
    n_states = 20  # mode-2 is divided into substates. each substate represents 100/n_states % progression in the mode
    mean_tray_full_time = 2912.2#2500.0 # seconds, this can vary from picker to picker
    _rate = (1.0 / mean_tray_full_time) * n_states

    # defining a state map to change through the substates. 0 --> 1 --> 2 --> 3 --> 4 --> 5 --> 0
    state_map = np.eye(n_states, k=1) # connect all the successive nodes
    state_map[-1,0] = 1
    state_map += np.eye(n_states, k=-1)*0.1 # A small probability to go back, connect all the successive nodes reverse
    state_map[0,-1] = 0.1

    rs = np.sum(state_map, axis=1)
    print('rs',rs)

    # transition rate matrix
    Q = (np.diag(-rs) + state_map)*_rate
    print ('Q', Q)

    # creating observation matrix, assuming each states has ~70% prob to emit the state itself as observation
    # and another ~10% for neighbouring states each (confusing them). and +.1% for all observations
    # for numerical stability
    B_pre = np.ones(n_states) * .001 + np.eye(n_states) * .7 + np.eye(n_states, k=1) * .1 + np.eye(n_states, k=-1) * .1
    print ('B_pre', B_pre)
    # adding 10% change of "unknown" observation which each state is equally likely to emit (used for prediction)
    B = np.transpose(np.vstack([B_pre, [.101] * n_states]))  # np.vstack will add extra column in B_pre matrix vertically
    print ('B',B)

    B[0,-2] = .101     # first row and second last column is filled with 0.101
    B[-1,0] = .101     # Last row and first columm is filled with 0.101
    print ('B', B)

    # normalise B (make sure probs sum up to 1)
    row_sums = B.sum(axis=1)
    B = B / row_sums[:, np.newaxis]
    print ('B', B)

    # Pi is the vector of initial state probabilities. Assuming uniform here
    # (We may make a stronger assumption here at some point)
    Pi = np.array([1.0 / n_states] * n_states )   # Uniform Pi for all substates
    # Non-Uniform Pi
#    Pi = np.zeros((n_states, 1))
#    Pi[0,0] = 1.0


    # Create CtHMM by given parameters.
    mode2_model = HMModel(n_states, False, None, Q, B, Pi)
    # save model
    mode2_model.to_file("mode2_model")
    # load model from file
    mode2_model = HMModel(n_states, True, "mode2_model.npz")

    # sample a random sequence within desired time peroiod from the above created model(for testing and generation)
    t_seq, s_seq, e_seq = mode2_model.generate_random(sample_len=3000, sample_step=1)


#    #print(s_seq)

    # predict for a specific time from an initial observation
    (state, KL, posteriors) = mode2_model.predict(
                                                 # start with some observations assumed to have made up to a point
                                                 obs=np.array([0, 0, 0]),
                                                 # the time horizon to predict to
                                                 predict_time=1700,
                                                 # we want to see stuff here
                                                 verbose=True
                                                 )

    # forecast max seconds
    times, states, kls, posteriors = mode2_model.check_prediction_probs(obs=[2, 3, 3, 3, 3, 3], forecast_max=3600, forecast_steps=100, verbose=True)
    print (sorted(states.items(), key=operator.itemgetter(0)))


# The hm models below model the transition of the pickers along the topological map(nodes), and could be used
# to predict the node at which the picker would be, from an initial observation.
# these models should be similar to the hm model section above, using the HMModel class

#==============================================================================
# FORWARD PICKING : mode2_node_model
#==============================================================================
    print ('\n FORWARD PICKING : fwd_picknav_model \n')
    #State2 consists of several node transition in a unidirectional way in forward direction:

    n_states = 196  #number of nodes considered (2 ROW with 96 row_nodes each and 1, 1 head_nodes and sec_nodes, hence each ROW = 100 nodes)
                    # Each row is paralle yet in a cyclic pattern as (HOW TOPO_MAP LOOKS like :  ---><pri-hn-00 ---><--- 1---><---2..
                    # --><--96 ---><---sec-hn-00---><---sec-hn-01....  ---><--- rn-01-01---><---rn-01-00 ---><---pri-hn-01---><pri-hn-00.
    # Hence the idea is to create two identity matrices and concatenate it also keep the cyclic pattern by joining first and last node
    # defining a very simple state map -- Here state map = topo_node pattern.

    state_map = np.eye(n_states, k=1) # forward movement from row 1-->2 in a cyclic pattern (Counter Clock-Wise)
    state_map[97:99,97:99] = 0            # [ (total_rows/2)-1: (total_rows/2)+1 ] ; note: row count starts with 0
    state_map[98,0] = 1                   # connection between first node of first row and second node of second node in forward move [ (total_rows/2): 0]
    state_map[0,98] = 1                   # connection between first node of first row and second node of second node in reverse move [ 0: (total_rows/2)]


    state_map += np.eye(n_states, k=-1)*0.1  # reverse movement from row 2-->1 in a cyclic pattern (Clock-Wise)
    state_map[97:99,97:99] = 0               # [ (total_rows/2)-1: (total_rows/2)+1 ] ; note: row count starts with 0
    state_map[97,-1] = 1                     # connection between last node of first row and last node of second node in forward move.[(total_rows/2)-1: -1]
    state_map[-1,97] = 1                     #connection between first node of first row and second node of second node in reverse move.[-1: (total_rows/2)-1]

#    print (state_map)


    # summing all column of adjency matrix
    rs = np.sum(state_map, 1)       # it sums up all the columns of a single row,so that it can help in defining Q in next step

    # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
    # expected mean rate in seconds
    _rate =   0.001724078  # The rate is calculated from DES
    _lambda = _rate
#    _lambda = 1.0/_rate

    Q = (np.diag(-rs) + state_map) * _lambda   # Keep in mind that, sum(Qij) = -Qii =< 1.


    # MODE = 2
    # creating observation matrix based on DES data, each node has ~1.0% prob to emit the state itself as observation
    # and another ~97% for neighbouring nodes in forward direction and ~1% for neighbouring nodes in reverse direction each.
    # and +.1% for all observations for numerical stability

#    B_pre = np.ones(n_states) * .001 + np.eye(n_states) * .01 + np.eye(n_states, k=1) * .97 + np.eye(n_states, k=-1) * .01
    B_pre = np.ones(n_states) * .001 + np.eye(n_states) * .7 + np.eye(n_states, k=1) * .02 + np.eye(n_states, k=-1) * .02 + np.eye(n_states, k=2) * .01 + np.eye(n_states, k=-2) * .01


    B = np.transpose(np.vstack([B_pre, [.101] * n_states]))  # np.vstack will add extra column in B_pre matrix
                                                            # vertically
    B[0,-2] = .101     # first row and second last column is filled with 0.101

    B[-1,0] = .101     # Last row and first columm is filled with 0.101

    # normalise B (make sure probs sum up to 1)
    row_sums = B.sum(axis=1)
    B = B / row_sums[:, np.newaxis]
    # Pi is the vector of initial state probabilities. Assuming uniform here
    # (We may make a stronger assumption here at some point)
    Pi = np.array([1.0 / n_states] * n_states )   # Uniform Pi for all substates
#    Pi[168] = 1
    # Create CtHMM by given parameters.
    fwd_picknav_model = HMModel(n_states, False, None, Q, B, Pi)
    # save model
    fwd_picknav_model.to_file("fwd_picknav_model")

    # load model from file
    fwd_picknav_model = HMModel(n_states, True, "fwd_picknav_model.npz")

    # sample a random sequence within desired time peroiod from the above created model(for testing and generation)
    t_seq, s_seq, e_seq = fwd_picknav_model.generate_random(sample_len=2000, sample_step=1)

    # predict for a specific time from an initial observation
    (state, KL, posteriors) = fwd_picknav_model.predict(
                                                 # start with some observations assumed to have made up to a point
                                                 obs=np.array([31,31,31]),
                                                 # the time horizon to predict to
                                                 predict_time=1500,
                                                 # we want to see stuff here
                                                 verbose=True
                                                 )

    # forecast max seconds
    times, states, kls, posteriors = fwd_picknav_model.check_prediction_probs(obs=[31,32,33], forecast_max=3000., forecast_steps=200, verbose=True)
    print (sorted(states.items(), key=operator.itemgetter(0)))

#==============================================================================
# BACKWARD PICKING : mode2_node_model
#==============================================================================
    print ('\n BACKWARD PICKING : bwd_picknav_model \n')
    #State2 consists of several node transition in a unidirectional way in backward direction:

    n_states = 196  #number of nodes considered (2 ROW with 96 row_nodes each and 1, 1 head_nodes and sec_nodes, hence each ROW = 100 nodes)
                    # Each row is paralle yet in a cyclic pattern as (HOW TOPO_MAP LOOKS like :  ---><pri-hn-00 ---><--- 1---><---2..
                    # --><--96 ---><---sec-hn-00---><---sec-hn-01....  ---><--- rn-01-01---><---rn-01-00 ---><---pri-hn-01---><pri-hn-00.
    # Hence the idea is to create two identity matrices and concatenate it also keep the cyclic pattern by joining first and last node
    # defining a very simple state map -- Here state map = topo_node pattern.

    state_map = np.eye(n_states, k=1)*0.1 # forward movement from row 1-->2 in a cyclic pattern (Counter Clock-Wise)
    state_map[97:99,97:99] = 0            # [ (total_rows/2)-1: (total_rows/2)+1 ] ; note: row count starts with 0
    state_map[98,0] = 1                   # connection between first node of first row and second node of second node in forward move [ (total_rows/2): 0]
    state_map[0,98] = 1                   # connection between first node of first row and second node of second node in reverse move [ 0: (total_rows/2)]


    state_map += np.eye(n_states, k=-1)  # reverse movement from row 2-->1 in a cyclic pattern (Clock-Wise)
    state_map[97:99,97:99] = 0               # [ (total_rows/2)-1: (total_rows/2)+1 ] ; note: row count starts with 0
    state_map[97,-1] = 1                     # connection between last node of first row and last node of second node in forward move.[(total_rows/2)-1: -1]
    state_map[-1,97] = 1                     #connection between first node of first row and second node of second node in reverse move.[-1: (total_rows/2)-1]
#    print (state_map)


    # summing all column of adjency matrix
    rs = np.sum(state_map, 1)       # it sums up all the columns of a single row,
                                  #so that it can help in defining Q in next step

    # creating the transition rate matrix (https://en.wikipedia.org/wiki/Transition_rate_matrix)
    # DONE : TODO: transition rates are not constant across the nodes hence need to be cal.
    # expected mean rate in seconds
    _rate =  0.001724078  #The rate is calculated from DES
    _lambda = _rate
#    _lambda = 1.0/_rate

    Q = (np.diag(-rs) + state_map) * _lambda   # Keep in mind that, sum(Qij) = -Qii =< 1.


    # MODE = 2
    # creating observation matrix based on DES data, each node has ~1.0% prob to emit the state itself as observation
    # and another ~97% for neighbouring nodes in forward direction and ~1% for neighbouring nodes in reverse direction each.
    # and +.1% for all observations for numerical stability

#    B_pre = np.ones(n_states) * .001 + np.eye(n_states) * .01 + np.eye(n_states, k=1) * .97 + np.eye(n_states, k=-1) * .01
    B_pre = np.ones(n_states) * .001 + np.eye(n_states) * .7 + np.eye(n_states, k=1) * .02 + np.eye(n_states, k=-1) * .02 + np.eye(n_states, k=2) * .01 + np.eye(n_states, k=-2) * .01


    B = np.transpose(np.vstack([B_pre, [.101] * n_states]))  # np.vstack will add extra column in B_pre matrix
                                                            # vertically
    B[0,-2] = .101     # first row and second last column is filled with 0.101

    B[-1,0] = .101     # Last row and first columm is filled with 0.101

    # normalise B (make sure probs sum up to 1)
    row_sums = B.sum(axis=1)
    B = B / row_sums[:, np.newaxis]
    # Pi is the vector of initial state probabilities. Assuming uniform here
    # (We may make a stronger assumption here at some point)
    Pi = np.array([1.0 / n_states] * n_states )   # Uniform Pi for all substates
#    Pi[168] = 1
    # Create CtHMM by given parameters.
    bwd_picknav_model = HMModel(n_states, False, None, Q, B, Pi)
    # save model
    bwd_picknav_model.to_file("bwd_picknav_model")

    # load model from file
    bwd_picknav_model = HMModel(n_states, True, "bwd_picknav_model.npz")

    # sample a random sequence within desired time peroiod from the above created model(for testing and generation)
    t_seq, s_seq, e_seq = bwd_picknav_model.generate_random(sample_len=5000, sample_step=1)

    # predict for a specific time from an initial observation
    (state, KL, posteriors) = bwd_picknav_model.predict(
                                                 # start with some observations assumed to have made up to a point
                                                 obs=np.array([193,193,193]),
                                                 # the time horizon to predict to
                                                 predict_time=3000,
                                                 # we want to see stuff here
                                                 verbose=True
                                                 )

    # forecast max seconds
    times, states, kls, posteriors = bwd_picknav_model.check_prediction_probs(obs=[191,192,193], forecast_max=3000., forecast_steps=200, verbose=True)
    print (sorted(states.items(), key=operator.itemgetter(0)))
