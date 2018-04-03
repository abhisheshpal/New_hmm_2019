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
import rasberry_des.topo

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

    rospy.init_node("des", anonymous=False)
    # required des config parameters
    config_params = rasberry_des.config_utils.get_des_config_parameters(map_from_db=False)

    n_farm_rows = config_params["n_farm_rows"]
    half_rows = config_params["half_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]

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

    picker_ids = ["picker_%02d" %(i) for i in range(n_pickers)]

    picking_rate = rasberry_des.config_utils.param_list_to_dict("picking_rate", _picking_rate, picker_ids)
    picker_transportation_rate = rasberry_des.config_utils.param_list_to_dict("picker_transportation_rate", _picker_transportation_rate, picker_ids)
    picker_max_n_trays = rasberry_des.config_utils.param_list_to_dict("picker_max_n_trays", _picker_max_n_trays, picker_ids)
    picker_unloading_time = rasberry_des.config_utils.param_list_to_dict("picker_unloading_time", _picker_unloading_time, picker_ids)

    robot_ids = ["robot_%02d" %(i) for i in range(n_robots)]

    robot_transportation_rate = rasberry_des.config_utils.param_list_to_dict("robot_transportation_rate", _robot_transportation_rate, robot_ids)
    robot_max_n_trays = rasberry_des.config_utils.param_list_to_dict("robot_max_n_trays", _robot_max_n_trays, robot_ids)
    robot_unloading_time = rasberry_des.config_utils.param_list_to_dict("robot_unloading_time", _robot_unloading_time, robot_ids)

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

    # assuming a fork graph with a head lane
    local_storages = [simpy.Resource(rasb_env, capacity=n_pickers) for i in range(n_local_storages)]

    topo_graph = rasberry_des.topo.TopologicalForkGraph(n_farm_rows, half_rows,
                                                        n_topo_nav_rows,
                                                        _yield_per_node,
                                                        local_storages)

    rasb_robots = []
    for robot_id in robot_ids:
        rasb_robots.append(rasberry_des.robot.Robot(robot_id, robot_transportation_rate[robot_id],
                                                    robot_max_n_trays[robot_id],
                                                    robot_unloading_time[robot_id],
                                                    rasb_env, topo_graph))

    rasb_pickers = []
    for picker_id in picker_ids:
        rasb_pickers.append(rasberry_des.picker.Picker(picker_id, tray_capacity,
                                                       picker_max_n_trays[picker_id],
                                                       picking_rate[picker_id],
                                                       picker_transportation_rate[picker_id],
                                                       picker_unloading_time[picker_id],
                                                       rasb_env, topo_graph,
                                                       rasb_robots))

    # policy - "lexographical", "shortest_distance", "utilise_all"
    rasb_farm = rasberry_des.farm.Farm("RAS-Berry", rasb_env,
                                       n_topo_nav_rows, topo_graph,
#                                       rasb_robots, rasb_pickers, "lexographical")
#                                       rasb_robots, rasb_pickers, "shortest_distance")
                                       rasb_robots, rasb_pickers, "utilise_all")

    SHOW_VIS = True
    SHOW_INFO = True
    if SHOW_VIS:
        vis = rasberry_des.visualise.Visualise_Agents(topo_graph, rasb_robots, rasb_pickers)

#    rasb_env.run()
    while not rospy.is_shutdown():
        try:
            # instead of env.run() we should env.step() to have any control (Ctrl+c)
            # If multiple events are scheduled for the same simpy.time, there would be
            # at least a ms delay (in ros/realtime clock) between the events
            # This seems to be unavoidable at this stage
            rasb_env.step()
            if SHOW_VIS:
                vis.update_plot()
        except simpy.core.EmptySchedule:
            if SHOW_VIS:
                vis.close_plot()
            rospy.signal_shutdown("Quit")
            break
        except rospy.ROSInterruptException:
            break
        else:
            pass

    if SHOW_INFO:
        # no ros related calls here to ensure printing even when the pickers_only node is killed
        # farm details
        print("-----------------\n----%s----\n-----------------" %(rasb_farm.name))
        print("n_pickers: %d" %(len(rasb_pickers)))
        print("n_farm_rows: %d" %(topo_graph.n_farm_rows))
        print("n_topo_nav_rows: %d" %(topo_graph.n_topo_nav_rows))
        tot_yield = 0.
        for row_id in topo_graph.row_ids:
            print("  --%s--" %(row_id))
            row_start_node = topo_graph.row_info[row_id][1]
            row_end_node = topo_graph.row_info[row_id][2]
            row_start_y = topo_graph.get_node(row_start_node).pose.position.y
            row_end_y = topo_graph.get_node(row_end_node).pose.position.y
            row_length = row_end_y - row_start_y
            node_dist = topo_graph.get_distance_between_nodes(topo_graph.row_nodes[row_id][0], topo_graph.row_nodes[row_id][1])
            print("  row_length: %0.3f m" %(row_length))
            print("  node_dist: %0.3f m" %(node_dist))
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            if (not topo_graph.half_rows) and (row_id == "row-%02d" %(0) or row_id == "row-%02d" %(topo_graph.n_topo_nav_rows)):
                for i in range(n_row_nodes):
                    row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
            else:
                for i in range(n_row_nodes):
                    if (i == 0) or (i == n_row_nodes - 1):
                        row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
                    else:
                        row_yield += 2 * topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
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
