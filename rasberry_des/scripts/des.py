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
SAVE_STATS = True
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
        env = simpy.Environment()
    elif des_env == "ros":
        start_time_ros = rospy.get_time()
        # RealtimeEnvironment can be enabled by uncommenting the line below.
        # The farm size and n_pickers given would take 420s to run
        # To vary the speed of RT sim, change 'factor'
        env = simpy.RealtimeEnvironment(initial_time=0., factor=SIM_RT_FACTOR, strict=False)
    else:
        raise ValueError("%srasberry_des_config/des_env must be either simpy or ros" %(ns))
        rospy.logerr("%srasberry_des_config/des_env must be either simpy or ros" %(ns))

    start_time_simpy = env.now

    # assuming a fork graph with a head lane
    local_storages = [simpy.Resource(env, capacity=n_pickers) for i in range(n_local_storages)]

    topo_graph = rasberry_des.topo.TopologicalForkGraph(n_farm_rows, half_rows,
                                                        n_topo_nav_rows,
                                                        _yield_per_node,
                                                        local_storages)

    robots = []
    for robot_id in robot_ids:
        robots.append(rasberry_des.robot.Robot(robot_id, robot_transportation_rate[robot_id],
                                                    robot_max_n_trays[robot_id],
                                                    robot_unloading_time[robot_id],
                                                    env, topo_graph))

    pickers = []
    for picker_id in picker_ids:
        pickers.append(rasberry_des.picker.Picker(picker_id, tray_capacity,
                                                       picker_max_n_trays[picker_id],
                                                       picking_rate[picker_id],
                                                       picker_transportation_rate[picker_id],
                                                       picker_unloading_time[picker_id],
                                                       env, topo_graph,
                                                       robots))

    map_name = "open_field"     # "fork_map" or "open_field"
    scheduling_policy = "lexographical"     # "lexographical", "shortest_distance", "utilise_all"
    farm = rasberry_des.farm.Farm(map_name,
                                  env, n_topo_nav_rows, topo_graph, robots, pickers,
                                  scheduling_policy)

    if SHOW_VIS:
        vis = rasberry_des.visualise.Visualise_Agents(topo_graph, robots, pickers)

#    env.run()
    while not rospy.is_shutdown():
        try:
            # instead of env.run() we should env.step() to have any control (Ctrl+c)
            # If multiple events are scheduled for the same simpy.time, there would be
            # at least a ms delay (in ros/realtime clock) between the events
            # This seems to be unavoidable at this stage
            env.step()
            if SHOW_VIS:
                vis.update_plot()
        except simpy.core.EmptySchedule:
            if SHOW_VIS:
                vis.close_plot()
            rospy.signal_shutdown("Quit")
            break
        except rospy.ROSInterruptException:
            if SHOW_VIS:
                vis.close_plot()
            break
        else:
            pass

    finish_time_ros = rospy.get_time()
    finish_time_simpy = env.now

    if SAVE_STATS:
        import os
        f_handle = open(os.path.expanduser("~")+"/M%s_P%d_R%d.dat" %(map_name, n_pickers, n_robots), "w")
        # no ros related calls here to ensure printing even when the pickers_only node is killed
        # farm details
        print >> f_handle, "-----------------\n----%s----\n-----------------" %(farm.name)

        print >> f_handle, "simulation_start_time: %0.3f" %(start_time_simpy)
        print >> f_handle, "simulation_finish_time: %0.3f" %(finish_time_simpy)

        print >> f_handle, "n_pickers: %d" %(n_pickers)
        print >> f_handle, "n_robots: %d" %(n_robots)

        print >> f_handle, "n_farm_rows: %d" %(topo_graph.n_farm_rows)
        print >> f_handle, "n_topo_nav_rows: %d" %(topo_graph.n_topo_nav_rows)

        tot_yield = 0.
        for row_id in topo_graph.row_ids:
            print >> f_handle, "  --%s--" %(row_id)
            row_start_node = topo_graph.row_info[row_id][1]
            row_end_node = topo_graph.row_info[row_id][2]
            row_start_y = topo_graph.get_node(row_start_node).pose.position.y
            row_end_y = topo_graph.get_node(row_end_node).pose.position.y
            row_length = row_end_y - row_start_y
            node_dist = topo_graph.get_distance_between_nodes(topo_graph.row_nodes[row_id][0], topo_graph.row_nodes[row_id][1])
            print >> f_handle, "  row_length: %0.3f m" %(row_length)
            print >> f_handle, "  node_dist: %0.3f m" %(node_dist)
            row_yield = 0.
            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
            if (not topo_graph.half_rows) and (row_id == "row-%02d" %(0) or row_id == "row-%02d" %(topo_graph.n_topo_nav_rows)):
                for i in range(n_row_nodes - 1):
                    row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
            else:
                for i in range(n_row_nodes):
                    if (i == 0) or (i == n_row_nodes - 1):
                        row_yield += topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
                    else:
                        row_yield += 2 * topo_graph.yield_at_node[topo_graph.row_nodes[row_id][i]]
            print >> f_handle, "  row_yield: %0.3f g" %(row_yield)
            tot_yield += row_yield
        print >> f_handle, "tot_yield: %0.3f trays (%0.3f g)" %(tot_yield/tray_capacity, tot_yield)
        print >> f_handle, "\n"

        # picker details
        for i in range(n_pickers):
            print >> f_handle, "----%s----\n-----------------" %(pickers[i].picker_id)
            print >> f_handle, "picking_rate: %0.3f m/s" %(pickers[i].picking_rate)
            print >> f_handle, "picker_transportation_rate: %0.3f m/s" %(pickers[i].transportation_rate)
            print >> f_handle, "tray_capacity: %d g" %(pickers[i].tray_capacity)
            print >> f_handle, "picker_max_n_trays: %d" %(pickers[i].max_n_trays)
            print >> f_handle, "rows allocated: ", farm.picker_allocations[pickers[i].picker_id]
            for row_id in farm.picker_allocations[pickers[i].picker_id]:
                alloc_time = farm.allocation_time[row_id]
                finish_time = farm.row_finish_time[row_id]
                print >> f_handle, "  %s allocation time: %0.3f" %(row_id,
                                                      alloc_time if alloc_time is not None else float("inf"))
                print >> f_handle, "  %s completion time: %0.3f" %(row_id,
                                                      finish_time if finish_time is not None else float("inf"))
            print >> f_handle, "tot_trays: %0.3f (%0.3f g)" %(pickers[i].tot_trays,
                                                 pickers[i].tot_trays * pickers[i].tray_capacity)
            print >> f_handle, "picking_time: %0.3f" %(pickers[i].time_spent_picking)
            print >> f_handle, "transportation_time: %0.3f" %(pickers[i].time_spent_transportation)
            print >> f_handle, "idle_time: %0.3f" %(pickers[i].time_spent_idle)
            if n_robots > 0:
                print >> f_handle, "waiting_for_robot_time: %0.3f" %(pickers[i].time_spent_waiting)
                print >> f_handle, "loading_on_robot_time: %0.3f" %(pickers[i].time_spent_loading)
            print >> f_handle, "unloading_time: %0.3f" %(pickers[i].time_spent_unloading)
            print >> f_handle, "total_working_time: %0.3f" %(pickers[i].time_spent_working())
            print >> f_handle, "-----------------\n"

            # robot details
            for i in range(n_robots):
                print >> f_handle, "----%s----\n-----------------" %(robots[i].robot_id)
                print >> f_handle, "robot_transportation_rate: %0.3f m/s" %(robots[i].transportation_rate)
                print >> f_handle, "robot_max_n_trays: %d" %(robots[i].max_n_trays)
                print >> f_handle, "tot_trays: %0.3f" %(robots[i].tot_trays)
                print >> f_handle, "picking_time: %0.3f" %(robots[i].time_spent_picking)
                print >> f_handle, "transportation_time: %0.3f" %(robots[i].time_spent_transportation)
                print >> f_handle, "idle_time: %0.3f" %(robots[i].time_spent_idle)
                print >> f_handle, "loading_time: %0.3f" %(robots[i].time_spent_loading)
                print >> f_handle, "unloading_time: %0.3f" %(robots[i].time_spent_unloading)
                print >> f_handle, "charging_time: %0.3f" %(robots[i].time_spent_charging)
                print >> f_handle, "total_working_time: %0.3f" %(robots[i].time_spent_working())
                print >> f_handle, "-----------------\n"
        f_handle.close()
