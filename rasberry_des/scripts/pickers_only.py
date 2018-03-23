#!/usr/bin/env python
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

import random
import simpy
import numpy
import sys
import rospy
import rasberry_des.farm
import rasberry_des.picker
import rasberry_des.config_utils
import rasberry_des.visualise
import rasberry_des.robot

RANDOM_SEED = 1234
SHOW_VIS = True
DETAILED_VIS = False
SHOW_INFO = False
SIM_RT_FACTOR = 10.0


if __name__ == "__main__":
    random.seed(RANDOM_SEED)
    numpy.random.seed(RANDOM_SEED)

    ns = rospy.get_namespace()

    if len(sys.argv) < 2:
        top_map = "fork_map"
    else:
        top_map = sys.argv[1]

    rospy.init_node("pickers_only", anonymous=True)
    # required des config parameters
    config_params = rasberry_des.config_utils.get_des_config_parameters(map_from_db=False)

    n_farm_rows = config_params["n_farm_rows"]
    half_rows = config_params["half_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]
    head_row_node_dist = config_params["head_row_node_dist"]
    head_node_y = config_params["head_node_y"]
    _row_node_dist = config_params["row_node_dist"]
    _row_length = config_params["row_length"]
    _row_spacing = config_params["row_spacing"]

    des_env = config_params["des_env"]
    n_pickers = config_params["n_pickers"]
    _picking_rate = config_params["picking_rate"]
    _picker_transportation_rate = config_params["picker_transportation_rate"]
    _picker_max_n_trays = config_params["picker_max_n_trays"]
    _picker_unloading_time = config_params["picker_unloading_time"]
    tray_capacity = config_params["tray_capacity"]
    n_local_storages = config_params["n_local_storages"]

    _yield_per_node = config_params["yield_per_node"]

    n_robots = config_params["n_robots"]
    _robot_transportation_rate = config_params["robot_transportation_rate"]
    _robot_max_n_trays = config_params["robot_max_n_trays"]
    _robot_unloading_time = config_params["robot_unloading_time"]

    if des_env == "simpy":
        rasb_env = simpy.Environment()
    elif des_env == "ros":
        t_start = rospy.get_time()
        # RealtimeEnvironment can be enabled by uncommenting the line below.
        # The farm size and n_pickers given would take 420s to run
        # To vary the speed of RT sim, change 'factor'
        rasb_env = simpy.RealtimeEnvironment(initial_time=t_start, factor=SIM_RT_FACTOR, strict=False)
    else:
        raise ValueError("%srasberry_des_config/des_env must be either simpy or ros" %(ns))
        rospy.logerr("%srasberry_des_config/des_env must be either simpy or ros" %(ns))

    picker_ids = ["picker_%02d" %(i) for i in range(n_pickers)]
    robot_ids = ["robot_%02d" %(i) for i in range(n_robots)]

    # assuming a fork graph with the following:
    # 1. only one head lane
    # 2. a picker won't have to wait longer than loadingTime
    local_storages = [simpy.Resource(rasb_env, capacity=n_pickers) for i in range(n_local_storages)]
    rasb_farm = rasberry_des.farm.Farm("RAS-Berry", rasb_env, des_env, n_farm_rows, half_rows,
                                       _yield_per_node, local_storages, picker_ids, robot_ids)

#    rasb_farm.init_graph_fork(n_farm_rows, half_rows, n_topo_nav_rows,
#                              head_row_node_dist, head_node_y,
#                              _row_node_dist, _row_length, _row_spacing,
#                              _yield_per_node, local_storages)

    rasb_env.process(rasb_farm.scheduler_monitor())

    if _picking_rate.__class__ == list:
        if len(_picking_rate) == n_pickers:
            picking_rate = {picker_ids[i]:_picking_rate[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            picking_rate = {picker_ids[i]:random.uniform(_picking_rate[0] - 0.04, _picking_rate[0] + 0.04) for i in range(n_pickers)}
        else:
            rospy.ROSException("picking_rate must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("picking_rate must be a list of size %d or 1" %(n_pickers))

    if _picker_transportation_rate.__class__ == list:
        if len(_picker_transportation_rate) == n_pickers:
            picker_transportation_rate = {picker_ids[i]:_picker_transportation_rate[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            picker_transportation_rate = {picker_ids[i]:random.uniform(_picker_transportation_rate[0] - 0.08, _picker_transportation_rate[0] + 0.08) for i in range(n_pickers)}
        else:
            rospy.ROSException("picker_transportation_rate must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("picker_transportation_rate must be a list of size %d or 1" %(n_pickers))

    if _picker_max_n_trays.__class__ == list:
        if len(_picker_max_n_trays) == n_pickers:
            picker_max_n_trays = {picker_ids[i]:_picker_max_n_trays[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            picker_max_n_trays = {picker_ids[i]:_picker_max_n_trays[0] for i in range(n_pickers)}
        else:
            rospy.ROSException("picker_max_n_trays must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("picker_max_n_trays must be a list of size %d or 1" %(n_pickers))

    if _picker_unloading_time.__class__ == list:
        if len(_picker_unloading_time) == n_pickers:
            picker_unloading_time = {picker_ids[i]:_picker_unloading_time[i] for i in range(n_pickers)}
        elif len(_picking_rate) == 1:
            picker_unloading_time = {picker_ids[i]:random.uniform(_picker_unloading_time[0] - 4, _picker_unloading_time[0] + 4) for i in range(n_pickers)}
        else:
            rospy.ROSException("picker_unloading_time must be a list of size %d or 1" %(n_pickers))
    else:
        rospy.ROSException("picker_unloading_time must be a list of size %d or 1" %(n_pickers))

    if _robot_transportation_rate.__class__ == list:
        if len(_robot_transportation_rate) == n_robots:
            robot_transportation_rate = {robot_ids[i]:_robot_transportation_rate[i] for i in range(n_robots)}
        elif len(_picking_rate) == 1:
            robot_transportation_rate = {robot_ids[i]:random.uniform(_robot_transportation_rate[0] - 4, _robot_transportation_rate[0] + 4) for i in range(n_robots)}
        else:
            rospy.ROSException("robot_transportation_rate must be a list of size %d or 1" %(n_robots))
    else:
        rospy.ROSException("robot_transportation_rate must be a list of size %d or 1" %(n_robots))

    if _robot_max_n_trays.__class__ == list:
        if len(_robot_max_n_trays) == n_robots:
            robot_max_n_trays = {robot_ids[i]:_robot_max_n_trays[i] for i in range(n_robots)}
        elif len(_picking_rate) == 1:
            robot_max_n_trays = {robot_ids[i]:_robot_max_n_trays for i in range(n_robots)}
        else:
            rospy.ROSException("robot_max_n_trays must be a list of size %d or 1" %(n_robots))
    else:
        rospy.ROSException("robot_max_n_trays must be a list of size %d or 1" %(n_robots))

    if _robot_unloading_time.__class__ == list:
        if len(_robot_unloading_time) == n_robots:
            robot_unloading_time = {robot_ids[i]:_robot_unloading_time[i] for i in range(n_robots)}
        elif len(_picking_rate) == 1:
            robot_unloading_time = {robot_ids[i]:_robot_transportation_rate[0] for i in range(n_robots)}
        else:
            rospy.ROSException("robot_unloading_time must be a list of size %d or 1" %(n_robots))
    else:
        rospy.ROSException("robot_unloading_time must be a list of size %d or 1" %(n_robots))

picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, farm, des_env, sim_rt_factor=1.0

                 robot_id, transportation_rate, max_n_trays, env, farm, des_env, sim_rt_factor=1.0

    rasb_pickers = []
    rasb_robots = []
    if des_env == "ros":
        for picker_id in picker_ids:
            rasb_pickers.append(rasberry_des.picker.Picker(picker_id, tray_capacity,
                                                           picker_max_n_trays[picker_id],
                                                           picking_rate[picker_id],
                                                           picker_transportation_rate[picker_id],
                                                           picker_unloading_time[picker_id],
                                                           rasb_env, rasb_farm, des_env,
                                                           SIM_RT_FACTOR))

        for robot_id in robot_ids:
            rasb_robots.append(rasberry_des.robot.Robot(robot_id, robot_transportation_rate[robot_id],
                                                        robot_max_n_trays[robot_id],
                                                        robot_unloadin_time[robot_id],
                                                        rasb_env, rasb_farm,
                                                        des_env, SIM_RT_FACTOR))
    elif des_env == "simpy":
        for picker_id in picker_ids:
            rasb_pickers.append(rasberry_des.picker.Picker(picker_id, tray_capacity,
                                                           picker_max_n_trays[picker_id],
                                                           picking_rate[picker_id],
                                                           picker_transportation_rate[picker_id],
                                                           picker_unloading_time[picker_id],
                                                           rasb_env, rasb_farm, des_env))

        for robot_id in robot_ids:
            rasb_robots.append(rasberry_des.robot.Robot(robot_id, robot_transportation_rate[robot_id],
                                                        robot_max_n_trays[robot_id],
                                                        unloading_time[robot_id],
                                                        rasb_env, rasb_farm, des_env))

    SHOW_VIS = True
    SHOW_INFO = True
    if SHOW_VIS:
        vis = rasberry_des.visualise.Visualise_Agents(rasb_farm, picker_ids, DETAILED_VIS)

    while not rospy.is_shutdown():
        try:
#            t_now = rospy.get_time()
#            print ("%0.3f, %0.3f" %(rasb_env.now, t_now))
            # instead of env.run() we should env.step() to have any control (Ctrl+c)
            # If multiple events are scheduled for the same simpy.time, there would be
            # at least a ms delay (in ros/realtime clock) between the events
            # This seems to be unavoidable at this stage
            rasb_env.step()
            if SHOW_VIS:
                vis.plot_update()
        except simpy.core.EmptySchedule:
            break
        except rospy.ROSInterruptException:
            break
        else:
            pass

    if SHOW_INFO:
        # no ros related calls here to ensure printing even when the pickers_only node is killed
        # farm details
        print("-----------------\n----%s----\n-----------------" %(rasb_farm.name))
        print("n_pickers: %d" %(len(rasb_farm.pickers_reported)))
        print("n_farm_rows: %d" %(rasb_farm.n_farm_rows))
        print("n_topo_nav_rows: %d" %(rasb_farm.n_topo_nav_rows))
        tot_yield = 0.
        for row_id in rasb_farm.row_ids:
            print("  --%s--" %(row_id))
            row_start_node = rasb_farm.graph.row_info[row_id][1]
            row_end_node = rasb_farm.graph.row_info[row_id][2]
            row_start_y = rasb_farm.graph.get_node(row_start_node).pose.position.y
            row_end_y = rasb_farm.graph.get_node(row_end_node).pose.position.y
            row_length = row_end_y - row_start_y
            node_dist = rasb_farm.graph.row_info[row_id][3]
            print("  row_length: %0.3f m" %(row_length))
            print("  node_dist: %0.3f m" %(node_dist))
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            if (not rasb_farm.half_rows) and (row_id == "row-%02d" %(0) or row_id == "row-%02d" %(rasb_farm.n_topo_nav_rows)):
                for i in range(n_row_nodes):
                    row_yield += rasb_farm.graph.yield_at_node[rasb_farm.graph.row_nodes[row_id][i]]
            else:
                for i in range(n_row_nodes):
                    if (i == 0) or (i == n_row_nodes - 1):
                        row_yield += rasb_farm.graph.yield_at_node[rasb_farm.graph.row_nodes[row_id][i]]
                    else:
                        row_yield += 2 * rasb_farm.graph.yield_at_node[rasb_farm.graph.row_nodes[row_id][i]]
            print("  row_yield: %0.3f g" %(row_yield))
            tot_yield += row_yield
        print("tot_yield: %0.3f trays (%0.3f g)" %(tot_yield/tray_capacity, tot_yield))
        print("\n")

        # picker details
        for i in range(n_pickers):
            print("----%s----\n-----------------" %(rasb_pickers[i].picker_id))
            print("picking_rate: %0.3f m/s" %(rasb_pickers[i].picking_rate))
            print("picker_transportation_rate: %0.3f m/s" %(rasb_pickers[i].transportation_rate))
            print("tray_capacity: %d g" %(rasb_pickers[i].tray_capacity))
            print("picker_max_n_trays: %d" %(rasb_pickers[i].max_n_trays))
            print("rows allocated: ", rasb_farm.picker_allocations[rasb_pickers[i].picker_id])
            for row_id in rasb_farm.picker_allocations[rasb_pickers[i].picker_id]:
                alloc_time = rasb_farm.allocation_time[row_id]
                finish_time = rasb_farm.row_finish_time[row_id]
                print("  %s allocation time: %0.3f" %(row_id,
                                                      alloc_time if alloc_time is not None else float("inf")))
                print("  %s completion time: %0.3f" %(row_id,
                                                      finish_time if finish_time is not None else float("inf")))
            print("tot_trays: %0.3f (%0.3f g)" %(rasb_pickers[i].tot_trays,
                                                 rasb_pickers[i].tot_trays * rasb_pickers[i].tray_capacity))
            print("-----------------\n")
