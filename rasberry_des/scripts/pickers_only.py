#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info: Farm - a simple strawberry farm class (farm.py)
#        Picker - a simple picker class (picker.py)
#        Uses simpy to simulate three processes
#           1. Farm.scheduler_monitor()
#           2. Picker.picking_process()
#           3. Picker.transport_process()
#        Local storage is a simpy.Resource, now with the capacity same as N_PICKERS
#        Uses the simple topological graph representation in topo.py
# ----------------------------------

import rospy
import rasberry_des.farm
import rasberry_des.picker
import rasberry_des.config_utils
import random
import simpy
import numpy
import sys

RANDOM_SEED = 1234
SHOW_INFO = False


if __name__ == "__main__":
    random.seed(RANDOM_SEED)
    numpy.random.seed(RANDOM_SEED)

    if len(sys.argv) < 2:
        ns = "/rasberry_des_config/"
        top_map = "fork_map"
    elif len(sys.argv) == 2:
        ns = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1] + "/"
        top_map = "fork_map"
    else:
        ns = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1] + "/"
        top_map = sys.argv[2]

    rospy.init_node("rasberry_des", anonymous=True)
    # required des config parameters
    config_params = rasberry_des.config_utils.get_des_config_parameters(ns, map_from_db=False)

    n_farm_rows = config_params[0]
    half_rows = config_params[1]
    n_topo_nav_rows = config_params[2]
    head_row_node_dist = config_params[3]
    head_node_y = config_params[4]
    _row_node_dist = config_params[5]
    _row_length = config_params[6]
    _row_spacing = config_params[7]

    des_env = config_params[8]
    n_pickers = config_params[9]
    _picking_rate = config_params[10]
    _transportation_rate = config_params[11]
    _max_n_trays = config_params[12]
    _loading_time = config_params[13]
    tray_capacity = config_params[14]

    _yield_per_node = config_params[15]

    if des_env == "simpy":
        print des_env
        simpy_env = simpy.Environment()
        print simpy_env
    elif des_env == "ros":
        print des_env
        t_start = rospy.get_time()
        # RealtimeEnvironment can be enabled by uncommenting the line below.
        # The farm size and n_pickers given would take 420s to run
        # To vary the speed of RT sim, change 'factor'
        simpy_env = simpy.RealtimeEnvironment(initial_time=t_start, factor=1.0, strict=False)
        print simpy_env
    else:
        raise ValueError("%sdes_env must be either simpy or ros" %(ns))
        rospy.logerr("%sdes_env must be either simpy or ros" %(ns))

    # assuming a fork graph with the following:
    # 1. only one head lane
    # 2. a picker won't have to wait longer than loadingTime
    local_storages = [simpy.Resource(simpy_env, capacity=n_pickers)]
    rasb_farm = rasberry_des.farm.Farm("RAS-Berry", simpy_env)

    rasb_farm.init_graph_fork(n_farm_rows, half_rows, n_topo_nav_rows,
                              head_row_node_dist, head_node_y,
                              _row_node_dist, _row_length, _row_spacing,
                              _yield_per_node, local_storages)

    simpy_env.process(rasb_farm.scheduler_monitor())

    picker_ids = ["picker-%02d" %(i) for i in range(n_pickers)]

    if _picking_rate.__class__ == list:
        if len(_picking_rate) == n_pickers:
            picking_rate = {picker_ids[i]:_picking_rate[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            picking_rate = {picker_ids[i]:random.uniform(_picking_rate[0] - 0.04, _picking_rate[0] + 0.04) for i in range(n_pickers)}
        else:
            rospy.ROSException("picking_rate must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("picking_rate must be a list of size %d or 1" %(n_pickers))

    if _transportation_rate.__class__ == list:
        if len(_transportation_rate) == n_pickers:
            transportation_rate = {picker_ids[i]:_transportation_rate[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            transportation_rate = {picker_ids[i]:random.uniform(_transportation_rate[0] - 0.08, _transportation_rate[0] + 0.08) for i in range(n_pickers)}
        else:
            rospy.ROSException("transportation_rate must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("transportation_rate must be a list of size %d or 1" %(n_pickers))

    if _max_n_trays.__class__ == list:
        if len(_max_n_trays) == n_pickers:
            max_n_trays = {picker_ids[i]:_max_n_trays[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            max_n_trays = {picker_ids[i]:_max_n_trays[0] for i in range(n_pickers)}
        else:
            rospy.ROSException("max_n_trays must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("max_n_trays must be a list of size %d or 1" %(n_pickers))

    if _loading_time.__class__ == list:
        if len(_loading_time) == n_pickers:
            loading_time = {picker_ids[i]:_loading_time[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            loading_time = {picker_ids[i]:random.uniform(_loading_time[0] - 4, _loading_time[0] + 4) for i in range(n_pickers)}
        else:
            rospy.ROSException("loading_time must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("loading_time must be a list of size %d or 1" %(n_pickers))


    rasb_pickers = []
    for picker_id in picker_ids:
        rasb_pickers.append(rasberry_des.picker.Picker(picker_id, simpy_env, rasb_farm, tray_capacity,
                                                       max_n_trays[picker_id], picking_rate[picker_id],
                                                       transportation_rate[picker_id], loading_time[picker_id]))

    while not rospy.is_shutdown():
        try:
#            t_now = rospy.get_time()
#            print ("%0.3f, %0.3f" %(env.now, t_now))
            # instead of env.run() we should env.step() to have any control (Ctrl+c)
            # If multiple events are scheduled for the same simpy.time, there would be
            # at least a ms delay (in ros/realtime clock) between the events
            # This seems to be unavoidable at this stage
            simpy_env.step()
        except simpy.core.EmptySchedule:
            break
        except rospy.ROSInterruptException:
            break
        else:
            pass

    if (SHOW_INFO):
        # farm details
        print("-----------------\n----%s----\n-----------------" %(rasb_farm.name))
        print("n_pickers: %d" %(len(rasb_farm.pickers_reported)))
        print("n_rows: %d" %(rasb_farm.n_rows))
        tot_yield = 0.
        for row_id in rasb_farm.rows:
            print("  --%s--" %(row_id))
            row_length = rasb_farm.graph.nodes[rasb_farm.row_info[row_id][2]].y
            node_dist = rasb_farm.row_info[row_id][3]
            print("  row_length: %0.3f m" %(row_length))
            print("  node_dist: %0.3f m" %(node_dist))
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            for i in range(n_row_nodes):
                if (i == 0) or (i == n_row_nodes - 1):
                    row_yield += rasb_farm.graph.nodes[rasb_farm.row_node_names[row_id][i]].yield_at_node
                else:
                    row_yield += 2 * rasb_farm.graph.nodes[rasb_farm.row_node_names[row_id][i]].yield_at_node
            print("  row_yield: %0.3f g" %(row_yield))
            tot_yield += row_yield
        print("tot_yield: %0.3f trays (%0.3f g)" %(tot_yield/tray_capacity, tot_yield))
        print("\n")

        # picker details
        for i in range(n_pickers):
            print("----%s----\n-----------------" %(rasb_pickers[i].name))
            print("picking_rate: %0.3f m/s" %(rasb_pickers[i].picking_rate))
            print("transportation_rate: %0.3f m/s" %(rasb_pickers[i].transportation_rate))
            print("tray_capacity: %d g" %(rasb_pickers[i].tray_capacity))
            print("max_n_trays: %d" %(rasb_pickers[i].max_n_trays))
            print("rows allocated: ", rasb_farm.picker_allocations[rasb_pickers[i].name])
            for row_id in rasb_farm.picker_allocations[rasb_pickers[i].name]:
                alloc_time = rasb_farm.allocation_time[row_id]
                finish_time = rasb_farm.row_finish_time[row_id]
                print("  %s allocation time: %0.3f" %(row_id,
                                                      alloc_time if alloc_time is not None else float("inf")))
                print("  %s completion time: %0.3f" %(row_id,
                                                      finish_time if finish_time is not None else float("inf")))
            print("tot_trays: %0.3f (%0.3f g)" %(rasb_pickers[i].tot_trays,
                                                 rasb_pickers[i].tot_trays * rasb_pickers[i].tray_capacity))
            print("-----------------\n")
