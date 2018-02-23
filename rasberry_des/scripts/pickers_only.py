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
import random
import simpy
import numpy


RANDOM_SEED = 1234

N_ROWS = 5                  # number of rows
ROW_NODE_DIST = 2           # m, distance between two ndoes in a row
ROW_LENGTH = 10              # m, length of a row
ROW_SPACING = 2             # m, spacing between two rows

N_PICKERS = 3               # total number of pickers
PICKING_RATE = 0.2          # m/s, speed of the picker while picking
TRANSPORT_RATE = 0.8        # m/s, speed of the picker while transporting
MAX_N_TRAYS = 1             # maximum number of trays that can be carried by the
                            # picker
LOADING_TIME = 80.0         # s, time to be spent at the localStorage
TRAY_CAPACITY = 12 * 250.0  # g, tray is assumed to take 12 small trays, each 250 g

YIELD_PER_NODE = 200.0      # g/m, yield per node distance

SHOW_INFO = False


if __name__ == "__main__":
    random.seed(RANDOM_SEED)
    numpy.random.seed(RANDOM_SEED)

    rospy.init_node("rasberry_des", anonymous=True)
    
#    env = simpy.Environment()
    t_start = rospy.get_time()
    # RealtimeEnvironment can be enabled by uncommenting the line below.
    # The farm size and n_pickers given would take 420s to run
    # To vary the speed of RT sim, change 'factor'
    env = simpy.RealtimeEnvironment(initial_time=t_start, factor=1.0, strict=False)

    # assuming a fork graph with the following:
    # 1. only one head lane
    # 2. a picker won't have to wait longer than loadingTime
    local_storages = [simpy.Resource(env, capacity=N_PICKERS)]
    rasb_farm = rasberry_des.farm.Farm("RAS-Berry", env)
    rasb_farm.init_graph_fork(N_ROWS, ROW_NODE_DIST, ROW_LENGTH, ROW_SPACING, YIELD_PER_NODE, local_storages)
    env.process(rasb_farm.scheduler_monitor())

    rasb_pickers = []
    for i in range(N_PICKERS):
        picking_rate = random.uniform(PICKING_RATE - 0.04, PICKING_RATE + 0.04)
        transport_rate = random.uniform(TRANSPORT_RATE - 0.08, TRANSPORT_RATE + 0.08)
        loading_time = random.uniform(LOADING_TIME - 2, LOADING_TIME + 2)

        rasb_pickers.append(rasberry_des.picker.Picker("picker-%02d" %(i), env, rasb_farm, TRAY_CAPACITY,
                              MAX_N_TRAYS, picking_rate, transport_rate, loading_time))

    while not rospy.is_shutdown():
        try:
#            t_now = rospy.get_time()
#            print ("%0.3f, %0.3f" %(env.now, t_now))
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

    if (SHOW_INFO):        
        
        rospy.signal_shutdown("Simulation completed ")
    
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
        print("tot_yield: %0.3f trays (%0.3f g)" %(tot_yield/TRAY_CAPACITY, tot_yield))
        print("\n")
    
        # picker details
        for i in range(N_PICKERS):
            print("----%s----\n-----------------" %(rasb_pickers[i].name))
            print("picking_rate: %0.3f m/s" %(rasb_pickers[i].picking_rate))
            print("transport_rate: %0.3f m/s" %(rasb_pickers[i].transport_rate))
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
