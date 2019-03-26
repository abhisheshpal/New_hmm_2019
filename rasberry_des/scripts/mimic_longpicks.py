#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import random
import simpy
import numpy
import sys
import rospy
import rasberry_des.picker_mimic
import rasberry_des.config_utils_mimic
import rasberry_des.topo_mimic
import rasberry_des.farm_mimic

RANDOM_SEED = 1111
SIM_RT_FACTOR = 0.5
VERBOSE = True

random.seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)


#==============================================================================
# with_robots: false
#
# n_pickers: 3
# picker_picking_rate:
#   func: gauss
#   value: [0.2, 0.02]
# picker_transportation_rate:
#   func: gauss
#   value: [1.0, 0.04]
# picker_max_n_trays:
#   func: copy
#   value: [1]
# picker_unloading_time:
#   func: gauss
#   value: [10.0, 0.2]
# tray_capacity: 3000
#==============================================================================



if __name__ == "__main__":
    if len(sys.argv) <= 1:
        raise Exception("Not enough arguments. Pass 'path to configuration file' as an argument.")
    else:
        config_file = sys.argv[1]

    rospy.init_node("mimic_longpicks")
    rospy.sleep(3)

    # get the config params
    config_params = rasberry_des.config_utils_mimic.get_mimic_des_params(config_file)

    # create the topo_graph
    topo_graph = rasberry_des.topo_mimic.TopologicalForkGraphMimic(config_params["n_polytunnels"],
                                                                   config_params["n_farm_rows"],
                                                                   config_params["n_topo_nav_rows"],
                                                                   config_params["second_head_lane"],
                                                                   VERBOSE)
    # set row_info
    # local_storage_nodes should be set separately
    if config_params["second_head_lane"]:
        topo_graph.set_row_info(config_params["pri_head_nodes"], config_params["row_nodes"], config_params["sec_head_nodes"])
    else:
        rospy.loginfo(config_params["pri_head_nodes"])
        topo_graph.set_row_info(config_params["pri_head_nodes"], config_params["row_nodes"])
    # set node_yields
    topo_graph.set_node_yields(config_params["yield_per_node"])

    # pickers
    picker_ids = ["picker_%02d" %(i) for i in range(config_params["n_pickers"])]

    picker_picking_rate = rasberry_des.config_utils.param_list_to_dict("picker_picking_rate", config_params["picker_picking_rate"], picker_ids)
    picker_transportation_rate = rasberry_des.config_utils.param_list_to_dict("picker_transportation_rate", config_params["picker_transportation_rate"], picker_ids)
    picker_max_n_trays = rasberry_des.config_utils.param_list_to_dict("picker_max_n_trays", config_params["picker_max_n_trays"], picker_ids)
    picker_unloading_time = rasberry_des.config_utils.param_list_to_dict("picker_unloading_time", config_params["picker_unloading_time"], picker_ids)


#    TODO:
    robots = []

    env = simpy.RealtimeEnvironment(factor=SIM_RT_FACTOR, strict=False)

    local_storages = [simpy.Resource(env, capacity=config_params["n_pickers"]) for i in range(len(config_params["local_storage_nodes"]))]
    topo_graph.set_local_storages(local_storages, config_params["local_storage_nodes"])

    if config_params["use_cold_storage"]:
        cold_storage = simpy.Resource(env, capacity=config_params["n_pickers"])
        topo_graph.set_cold_storage(cold_storage, config_params["cold_storage_node"])


    pickers = []
    pickers = []
    for picker_id in picker_ids:
        pickers.append(rasberry_des.picker_mimic.PickerMimic(picker_id, config_params["tray_capacity"],
                                                             picker_max_n_trays[picker_id],
                                                             picker_picking_rate[picker_id],
                                                             picker_transportation_rate[picker_id],
                                                             picker_unloading_time[picker_id],
                                                             env, topo_graph,
                                                             robots, VERBOSE))

    scheduling_policy = "lexicographical" # ["lexicographical", "shortest_distance", "uniform_utilisation"]

    farm = rasberry_des.farm_mimic.FarmMimic(config_params["map_name"],
                                             env, config_params["n_topo_nav_rows"], topo_graph, robots,
                                             pickers, scheduling_policy, VERBOSE)

    # get config data
    # create n picker_mimic objects
    # loop n_repeat_picks with row allocation
    while not rospy.is_shutdown():
        try:
            # instead of env.run() we should env.step() to have any control (Ctrl+c)
            # If multiple events are scheduled for the same simpy.time, there would be
            # at least a ms delay (in ros/realtime clock) between the events
            # This seems to be unavoidable at this stage
            env.step()
        except simpy.core.EmptySchedule:
            break
        except rospy.ROSInterruptException:
            break
        else:
            pass

    finish_time_ros = rospy.get_time()
    finish_time_simpy = env.now