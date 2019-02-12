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
import os
import simpy
import numpy
import sys
import time
import rospy
import rasberry_des.farm
import rasberry_des.picker
import rasberry_des.config_utils
import rasberry_des.visualise
import rasberry_des.robot
import rasberry_des.topo

RANDOM_SEED = 1111
SHOW_VIS = False
SAVE_STATS = True
SIM_RT_FACTOR = 5.0
VERBOSE = False

random.seed(RANDOM_SEED)
numpy.random.seed(RANDOM_SEED)


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        raise Exception("Not enough arguments. Pass 'path to configuration file' as an argument.")
    else:
        config_file = sys.argv[1]

    rospy.init_node("des", anonymous=False)
    # required des config parameters
    config_params = rasberry_des.config_utils.get_des_config_parameters(config_file)

    map_name = config_params["map_name"]
    n_polytunnels = config_params["n_polytunnels"]
    n_farm_rows = config_params["n_farm_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]

    des_env = config_params["des_env"]
    second_head_lane = config_params["second_head_lane"]
    n_pickers = config_params["n_pickers"]
    tray_capacity = config_params["tray_capacity"]

    _yield_per_node = config_params["yield_per_node"]

    n_local_storages = config_params["n_local_storages"]

    topo_graph = rasberry_des.topo.TopologicalForkGraph(n_polytunnels, n_farm_rows,
                                                        n_topo_nav_rows, second_head_lane,
                                                        _yield_per_node, VERBOSE)

    n_trials = 1
    min_n_pickers = 3#1
    max_n_pickers = 4#n_topo_nav_rows + 1
    min_n_robots = 3#0
    max_n_robots = 4#max_n_pickers
#    n_local_storages = n_topo_nav_rows
#    policies = ["lexicographical", "shortest_distance", "uniform_utilisation"]
    policies = ["uniform_utilisation"]

    raw_input("Press ENTER to start the DES")

    for n_pickers in range(min_n_pickers, max_n_pickers):
        if rospy.is_shutdown():
            break
        for n_robots in range(min_n_robots, max_n_robots):
            if rospy.is_shutdown():
                break
            for scheduling_policy in policies:
                if rospy.is_shutdown():
                    break
                for trial in range(n_trials):
                    if rospy.is_shutdown():
                        break

                    # some config parameters need to be re-read with the n_robots and n_pickers
                    # as these parameters are returned as a list of size n_robots and n_pickers
                    config_params = rasberry_des.config_utils.get_des_config_parameters(config_file, n_pickers=n_pickers, n_robots=n_robots)

                    _picker_picking_rate = config_params["picker_picking_rate"]
                    _picker_transportation_rate = config_params["picker_transportation_rate"]
                    _picker_max_n_trays = config_params["picker_max_n_trays"]
                    _picker_unloading_time = config_params["picker_unloading_time"]

                    _robot_transportation_rate = config_params["robot_transportation_rate"]
                    _robot_max_n_trays = config_params["robot_max_n_trays"]
                    _robot_unloading_time = config_params["robot_unloading_time"]

                    picker_ids = ["picker_%02d" %(i) for i in range(n_pickers)]

                    picker_picking_rate = rasberry_des.config_utils.param_list_to_dict("picker_picking_rate", _picker_picking_rate, picker_ids)
                    picker_transportation_rate = rasberry_des.config_utils.param_list_to_dict("picker_transportation_rate", _picker_transportation_rate, picker_ids)
                    picker_max_n_trays = rasberry_des.config_utils.param_list_to_dict("picker_max_n_trays", _picker_max_n_trays, picker_ids)
                    picker_unloading_time = rasberry_des.config_utils.param_list_to_dict("picker_unloading_time", _picker_unloading_time, picker_ids)

                    robot_ids = ["robot_%02d" %(i) for i in range(n_robots)]

                    robot_transportation_rate = rasberry_des.config_utils.param_list_to_dict("robot_transportation_rate", _robot_transportation_rate, robot_ids)
                    robot_max_n_trays = rasberry_des.config_utils.param_list_to_dict("robot_max_n_trays", _robot_max_n_trays, robot_ids)
                    robot_unloading_time = rasberry_des.config_utils.param_list_to_dict("robot_unloading_time", _robot_unloading_time, robot_ids)

                    start_time_ros = rospy.get_time()

                    if des_env == "simpy":
                        env = simpy.Environment()
                    elif des_env == "ros":
                        # RealtimeEnvironment
                        # To vary the speed of RT sim, change 'factor'
                        env = simpy.RealtimeEnvironment(initial_time=0., factor=SIM_RT_FACTOR, strict=False)
                    else:
                        raise Exception("des_env must be either simpy or ros")

                    start_time_simpy = env.now

                    # assuming a fork graph with a head lane
                    local_storages = [simpy.Resource(env, capacity=n_pickers+n_robots) for i in range(n_local_storages)]
                    topo_graph.set_local_storages(local_storages)
                    cold_storage = simpy.Resource(env, capacity=n_pickers+n_robots)
                    topo_graph.set_cold_storage(cold_storage)

                    robots = []
                    for robot_id in robot_ids:
                        robots.append(rasberry_des.robot.Robot(robot_id, robot_transportation_rate[robot_id],
                                                               robot_max_n_trays[robot_id],
                                                               robot_unloading_time[robot_id],
                                                               env, topo_graph, VERBOSE))

                    pickers = []
                    for picker_id in picker_ids:
                        pickers.append(rasberry_des.picker.Picker(picker_id, tray_capacity,
                                                                  picker_max_n_trays[picker_id],
                                                                  picker_picking_rate[picker_id],
                                                                  picker_transportation_rate[picker_id],
                                                                  picker_unloading_time[picker_id],
                                                                  env, topo_graph,
                                                                  robots, VERBOSE))

                    farm = rasberry_des.farm.Farm(map_name,
                                                  env, n_topo_nav_rows, topo_graph, robots,
                                                  pickers, scheduling_policy, VERBOSE)

                    if SHOW_VIS:
                        vis = rasberry_des.visualise.VisualiseAgents(topo_graph, robots,
                                                                     pickers, scheduling_policy,
                                                                     show_cs=True,
                                                                     save_random=False,
                                                                     trial=trial)

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
                            break
                        except rospy.ROSInterruptException:
                            if SHOW_VIS:
                                vis.close_plot()
                            break
                        else:
                            pass

                    finish_time_ros = rospy.get_time()
                    finish_time_simpy = env.now

                    print n_pickers, n_robots, scheduling_policy, trial, finish_time_ros - start_time_ros

                    if SAVE_STATS:
                        time_now = time.time()*1000000

                        # event logs
                        f_handle = open(os.path.expanduser("~")+"/M%s_P%d_R%d_S%s_%d_events.dat" %(map_name, n_pickers, n_robots, scheduling_policy, time_now), "w")
                        for item in farm.events:
                            print >> f_handle, item
                        for picker_id in picker_ids:
                            print >> f_handle, picker_id
                            for item in farm.predictor.predictors[picker_id].modes_nodes_dirs_times:
                                print >> f_handle, item
                        f_handle.close()

                        # predictions log
                        f_handle = open(os.path.expanduser("~")+"/M%s_P%d_R%d_S%s_%d_predictions.dat" %(map_name, n_pickers, n_robots, scheduling_policy, time_now), "w")
                        print >> f_handle, "picker.pred_row, picker.pred_node, picker.pred_dir, picker.pred_time, picker.curr_node, picker.picking_dir, time_now"
                        print >> f_handle, "picker.prev_row, picker.curr_node, picker.picking_dir, time_now, actual\n"
                        for picker_id in picker_ids:
                            print >> f_handle, picker_id
                            predictions = farm.predictions[picker_id]
                            for tray in range(1, farm.tray_counts[picker_id] + 1):
#                                f_handle.write(predictions[tray])
                                print >> f_handle, "\t", tray, ":"
                                for item in predictions[tray]:
                                    print >> f_handle, "\t\t", item
                        f_handle.close()

                        # des logs
                        f_handle = open(os.path.expanduser("~")+"/M%s_P%d_R%d_S%s_%d.dat" %(map_name, n_pickers, n_robots, scheduling_policy, time_now), "w")
                        # no ros related calls here to ensure printing even when the pickers_only node is killed
                        # farm details
                        print >> f_handle, "-----------------\n----%s----\n-----------------" %(farm.name)

                        print >> f_handle, "simulation_finish_time(sim): %0.3f" %(finish_time_simpy)
                        print >> f_handle, "simulation_finish_time(clock): %0.3f" %(finish_time_ros - start_time_ros)

                        print >> f_handle, "n_pickers: %d" %(n_pickers)
                        print >> f_handle, "n_robots: %d" %(n_robots)

                        print >> f_handle, "n_polytunnels: %d" %(topo_graph.n_polytunnels)
                        for i in range(topo_graph.n_polytunnels):
                            print >> f_handle, "n_farm_rows[tunnel-%d]: %d" %(i, topo_graph.n_farm_rows[i])
                        print >> f_handle, "n_topo_nav_rows: %d" %(topo_graph.n_topo_nav_rows)

                        tot_yield = 0.
                        for row_id in topo_graph.row_ids:
                            print >> f_handle, "  --%s--" %(row_id)
                            row_start_node = topo_graph.row_info[row_id][1]
                            row_end_node = topo_graph.row_info[row_id][2]
                            row_start_y = topo_graph.get_node(row_start_node).pose.position.y
                            row_end_y = topo_graph.get_node(row_end_node).pose.position.y
                            row_length = row_end_y - row_start_y
                            node_dist = topo_graph.get_distance_between_adjacent_nodes(topo_graph.row_nodes[row_id][0], topo_graph.row_nodes[row_id][1])
                            print >> f_handle, "  row_length: %0.3f m" %(row_length)
                            print >> f_handle, "  node_dist: %0.3f m" %(node_dist)
                            row_yield = 0.
                            n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
                            if row_id in topo_graph.half_rows:
                                for i in range(1, n_row_nodes):
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
                            print >> f_handle, "picker_picking_rate: %0.3f m/s" %(pickers[i].picking_rate)
                            print >> f_handle, "picker_transportation_rate: %0.3f m/s" %(pickers[i].transportation_rate)
                            print >> f_handle, "tray_capacity: %d g" %(pickers[i].tray_capacity)
                            print >> f_handle, "picker_max_n_trays: %d" %(pickers[i].max_n_trays)
                            print >> f_handle, "picker_unloading_time(per tray): %d" %(pickers[i].unloading_time)
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
                            print >> f_handle, "robot_unloading_time(per tray): %d" %(robots[i].unloading_time)
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
