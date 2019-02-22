#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import sys
import yaml
import numpy
import matplotlib.pyplot
import math
import time

def get_node_yields(log_data, verbose=False):
    node_yields = {} # {node_id: yield}
    n_topo_nav_rows = log_data["env_details"]["n_topo_nav_rows"]
    assert n_topo_nav_rows == len(log_data["env_details"]["row_details"])
    row_ids = []
    row_nodes = {}
    for item in log_data["env_details"]["row_details"]:
        row_ids.append(item["row_id"])
        if verbose: print "row_id: %s" %(item["row_id"])
        if verbose: print "  row_nodes:"
        row_nodes[item["row_id"]] = {}
        for row_info in item["row_nodes"]:
            if verbose: print "    node_id:%s yield: %0.3f" %(row_info["node_id"], row_info["yield"])
            node_yields[row_info["node_id"]] = row_info["yield"]
    return node_yields

def get_allocated_rows(log_data, verbose=False):
    allocated_rows = {} # {picker_id: [allcoated_row_id_1, allcoated_row_id_2, ...]}
    n_pickers = log_data["sim_details"]["n_pickers"]
    assert n_pickers == len(log_data["sim_details"]["picker_states"])
    allocated_rows = {}
    for item in log_data["sim_details"]["picker_states"]:
        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  allocated_rows:"
        allocated_rows[item["picker_id"]] = []
        count = 1
        for alloc_rows in item["allocated_rows"]:
            if verbose: print "    %d: %s" %(count, alloc_rows["row_id"])
            allocated_rows[item["picker_id"]].append(alloc_rows["row_id"])
            count += 1
    return allocated_rows

def get_time_spent_in_rows(log_data, verbose=False):
    # Time spent in each row for all rows in each picker case
    n_pickers = log_data["sim_details"]["n_pickers"]
    assert n_pickers == len(log_data["sim_details"]["picker_states"])
    time_spent_in_rows = {}
    for item in log_data["sim_details"]["picker_states"]:
        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  allocated_rows:"
        time_spent_in_rows[item["picker_id"]] = {}
        count = 1
        for alloc_rows in item["allocated_rows"]:
            if verbose: print "   %d. %s: time_spent: %0.3f" %(count, alloc_rows["row_id"], alloc_rows["completion_time"] - alloc_rows["allocation_time"])
            time_spent_in_rows[item["picker_id"]][alloc_rows["row_id"]] = alloc_rows["completion_time"] - alloc_rows["allocation_time"]
            count += 1
    return time_spent_in_rows


def get_state_times(log_data, state, state_str, verbose=False):
    state_times = {}
    if verbose: print "MODE:%d - %s" %(state, state_str)
    for item in log_data["sim_details"]["picker_states"]:
        state_times[item["picker_id"]] = []
        state_start = None
        state_finish = None
        mean = 0
        sigma = 0
        count = 0
        for state_info in item["state_changes"]:
            if state_info["mode"] == state:
                if state_start is None:
                    state_start = state_info["time"]
                else:
                    pass
            else:
                if state_start is None:
                    pass
                else:
                    state_finish = state_info["time"]
                    state_times[item["picker_id"]].append(state_finish - state_start)
                    count += 1
                    state_start = None
                    state_finish = None


        mean = numpy.mean(state_times[item["picker_id"]])
        sigma = numpy.std(state_times[item["picker_id"]])

        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  transition count: %d, mean_time: %0.3f, std: %0.3f" %(count, mean, sigma)
        if verbose: print "  ", state_times[item["picker_id"]]

    return state_times

def get_multi_iter_state_time_gauss(state_times, state, state_str, verbose=False):
    gauss_distributions = {} # {picker: {mean:value, sigma:value}}
    # state_times = [{picker_01:[...], picker_02:[...]}]
    # assuming the number of pickers and picker ids are same in all iters
    picker_ids = state_times[0].keys()
    all_times = {}
    for picker_id in picker_ids:
        all_times[picker_id] = []
        gauss_distributions[picker_id] = {}

    # get all times from each picker
    for item in state_times:
        for picker_id in item:
            all_times[picker_id].extend(item[picker_id])

    for picker_id in picker_ids:
        mean = numpy.mean(all_times[picker_id])
        sigma = numpy.std(all_times[picker_id])

        gauss_distributions[picker_id]["mean"] = mean
        gauss_distributions[picker_id]["sigma"] = sigma

        if verbose:
            min_time = numpy.min(all_times[picker_id])
            max_time = numpy.max(all_times[picker_id])
            n, bins, patches = matplotlib.pyplot.hist(all_times[picker_id], bins=int(math.ceil((max_time-min_time)/(0.01*(max_time-min_time)))), range=(min_time, max_time))
            matplotlib.pyplot.xlabel('Time')
            matplotlib.pyplot.ylabel('No. of instances')
            matplotlib.pyplot.title("%d - %s" %(state, state_str))
            matplotlib.pyplot.show(block=False)
            matplotlib.pyplot.pause(1)
            time.sleep(1)
            matplotlib.pyplot.close()

            x = numpy.arange(min_time, max_time, .01)
            matplotlib.pyplot.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            matplotlib.pyplot.xlabel('Time(in msec)')
            matplotlib.pyplot.ylabel('Probability')
            matplotlib.pyplot.title('Prio Probability distribution-' "%d - %s" %(state, state_str))

            matplotlib.pyplot.show(block=False)
            matplotlib.pyplot.pause(1)
            time.sleep(1)
            matplotlib.pyplot.close()

    return gauss_distributions

def get_single_iter_state_time_gauss(state_times, state, state_str, verbose=False):
    gauss_distributions = {} # {picker: {mean:value, sigma:value}}
    # state_times = {picker_01:[...], picker_02:[...]}
    picker_ids = state_times.keys()

    for picker_id in picker_ids:
        gauss_distributions[picker_id] = {}

    for picker_id in picker_ids:
        mean = numpy.mean(state_times[picker_id])
        sigma = numpy.std(state_times[picker_id])

        gauss_distributions[picker_id]["mean"] = mean
        gauss_distributions[picker_id]["sigma"] = sigma

        if verbose:
            min_time = numpy.min(state_times[picker_id])
            max_time = numpy.max(state_times[picker_id])
            n, bins, patches = matplotlib.pyplot.hist(state_times[picker_id], bins=int(math.ceil((max_time-min_time)/(0.01*(max_time-min_time)))), range=(min_time, max_time))
            matplotlib.pyplot.xlabel('Time')
            matplotlib.pyplot.ylabel('No. of instances')
            matplotlib.pyplot.title("%d - %s" %(state, state_str))
            matplotlib.pyplot.show(block=False)
            matplotlib.pyplot.pause(1)
            time.sleep(1)
            matplotlib.pyplot.close()

            x = numpy.arange(min_time, max_time, .01)
            matplotlib.pyplot.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            matplotlib.pyplot.xlabel('Time(in msec)')
            matplotlib.pyplot.ylabel('Probability')
            matplotlib.pyplot.title('Prio Probability distribution-' "%d - %s" %(state, state_str))

            matplotlib.pyplot.show(block=False)
            matplotlib.pyplot.pause(1)
            time.sleep(1)
            matplotlib.pyplot.close()

    return gauss_distributions

if __name__ == "__main__":
    if len(sys.argv) <2:
        print ("usage: event_log_processor.py path_to_dir_with_logs")
        exit()

    n_trials = 0
    state_0_times = []
    state_1_times = []
    state_2_times = []
    state_3_times = []
    state_4_times = []

    logs_dir = os.path.abspath(sys.argv[1])
    for f_name in os.listdir(logs_dir):
        f_name_split = f_name.split("_")
        if not (os.path.isfile(logs_dir+"/"+f_name) and f_name_split[0][0] == "M"):
            continue

        print f_name

        f_handle = open(logs_dir+"/"+f_name, "r")
        try:
            log_data = yaml.load(f_handle)
        except:
            print "Error loading file %s as a YAML file\nProceeding to next file" %(f_name)
            f_handle.close()
            continue
        else:
            f_handle.close()

        n_trials += 1

        # node yield associated with each row
        node_yields = get_node_yields(log_data, verbose=False)

        # allocated rows of each picker
        allocated_rows = get_allocated_rows(log_data, verbose=False)

        # Time spent in each row for all rows in each picker case
        time_spent_in_row = get_time_spent_in_rows(log_data, verbose=False)

        # Get all state_0 times for all pickers
        state_0_times.append(get_state_times(log_data, 0, "Idle", verbose=True))

        # Get all state_1 times for all pickers
        state_1_times.append(get_state_times(log_data, 1, "Transport to row node", verbose=True))

        # Get all state_2 times for all pickers
        state_2_times.append(get_state_times(log_data, 2, "Picking", verbose=True))

        # Get all state_3 times for all pickers
        state_3_times.append(get_state_times(log_data, 3, "Transport to storage", verbose=True))

        # Get all state_4 times for all pickers
        state_4_times.append(get_state_times(log_data, 4, "Unload at storage", verbose=True))

    # single iter example
    gauss_0_0 = get_single_iter_state_time_gauss(state_0_times[0], 0, "Idle", verbose=True)

    # get gaussian distributions of state_0 times
    gauss_0 = get_multi_iter_state_time_gauss(state_0_times, 0, "Idle", verbose=True)

    # get gaussian distributions of state_1 times
    gauss_1 = get_multi_iter_state_time_gauss(state_1_times, 0, "Transport to row node", verbose=True)

    # get gaussian distributions of state_2 times
    gauss_2 = get_multi_iter_state_time_gauss(state_2_times, 0, "Picking", verbose=True)

    # get gaussian distributions of state_3 times
    gauss_3 = get_multi_iter_state_time_gauss(state_3_times, 0, "Transport to storage", verbose=True)

    # get gaussian distributions of state_4 times
    gauss_4 = get_multi_iter_state_time_gauss(state_4_times, 0, "Unload at storage", verbose=True)



