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

def get_node_yields(log_data, verbose=False):
    # to process log_data from a single iteration
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
    # to process log_data from a single iteration
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
    # to process log_data from a single iteration
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
    # to process log_data from a single iteration
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

def get_multi_iter_state_time_gauss(state_times, state, state_str, plot_data=False, verbose=False):
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
            fig = matplotlib.pyplot.figure()
            ax1 = fig.add_subplot(211)
            n, bins, patches = ax1.hist(all_times[picker_id], bins=int(math.ceil((max_time-min_time)/(0.01*(max_time-min_time)))), range=(min_time, max_time))
            ax1.set_xlabel('Time')
            ax1.set_ylabel('No. of instances')
            ax1.set_title("%d - %s" %(state, state_str))

            ax2 = fig.add_subplot(212)
            x = numpy.arange(min_time, max_time, .01)
            ax2.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Probability')
            ax2.set_title('Prio Probability distribution-' "%d - %s" %(state, state_str))
            fig.savefig("multi_iter distribution - state: %d - %s.png" %(state, state_str))
            fig.subplots_adjust(left=0.125, bottom=None, right=0.9, top=0.9, wspace=0.5, hspace=0.2)
            matplotlib.pyplot.close(fig)

    return gauss_distributions

def get_single_iter_state_time_gauss(state_times, state, state_str, plot_data=False):
    # to process log_data from a single iteration
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

        if plot_data:
            min_time = numpy.min(state_times[picker_id])
            max_time = numpy.max(state_times[picker_id])
            fig = matplotlib.pyplot.figure()
            ax1 = fig.add_subplot(211)
            n, bins, patches = ax1.hist(state_times[picker_id], bins=int(math.ceil((max_time-min_time)/(0.01*(max_time-min_time)))), range=(min_time, max_time))
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('No. of instances')
            ax1.set_title("%d - %s" %(state, state_str))

            ax2 = fig.add_subplot(212)
            x = numpy.arange(min_time, max_time, .01)
            ax2.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Probability')
            ax2.set_title('Prio Probability distribution-' "%d - %s" %(state, state_str))
            fig.savefig("single_iter distribution - state: %d - %s.png" %(state, state_str))
            fig.subplots_adjust(left=0.125, bottom=None, right=0.9, top=0.9, wspace=0.5, hspace=0.2)
            matplotlib.pyplot.close(fig)

    return gauss_distributions

def isclose(a, b, rel_tol=1e-06, abs_tol=0.0):
    """to check two floats a and b are close (nearly equal)
    """
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def get_tray_picking_times(log_data, verbose=False):
    # to process log_data from a single iteration
    state_times = {}
    if verbose: print "Picking Times per Tray"
    for item in log_data["sim_details"]["picker_states"]:
        state_times[item["picker_id"]] = []
        state_start = None
        state_finish = None
        state_delta = 0.
        mean = 0
        sigma = 0
        count = 0
        tray_started = False
        prev_state = None
        for state_info in item["state_changes"]:
            if tray_started:
                if state_info["mode"] == 0:
                    state_finish = state_info["time"]
                    state_delta += (state_finish - state_start)
                    prev_state = 0
                elif state_info["mode"] == 1:
                    prev_state = 1
                elif state_info["mode"] == 2:
                    if prev_state != 2:
                        state_start = state_info["time"]
                    prev_state = 2
                elif state_info["mode"] == 3:
                    state_finish = state_info["time"]
                    state_delta += (state_finish - state_start)
                    state_times[item["picker_id"]].append(state_delta)
                    count += 1
                    state_start = None
                    state_finish = None
                    state_delta = 0.
                    tray_started = False
                    prev_state = 3
                elif state_info["mode"] == 4:
                    prev_state = 4
                elif state_info["mode"] == 5:
                    state_finish = state_info["time"]
                    state_delta += (state_finish - state_start)
                    state_times[item["picker_id"]].append(state_delta)
                    count += 1
                    state_start = None
                    state_finish = None
                    state_delta = 0.
                    tray_started = False
                    prev_state = 5
                elif state_info["mode"] == 6:
                    prev_state = 6
            else:
                if state_info["mode"] == 0:
                    prev_state = 0
                elif state_info["mode"] == 1:
                    prev_state = 1
                elif state_info["mode"] == 2:
                    state_start = state_info["time"]
                    tray_started = True
                    prev_state = 2
                elif state_info["mode"] == 3:
                    prev_state = 3
                elif state_info["mode"] == 4:
                    prev_state = 4
                elif state_info["mode"] == 5:
                    prev_state = 5
                elif state_info["mode"] == 6:
                    prev_state = 6

        mean = numpy.mean(state_times[item["picker_id"]])
        sigma = numpy.std(state_times[item["picker_id"]])
        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  transition count: %d, mean_time: %0.3f, std: %0.3f" %(count, mean, sigma)
        if verbose: print "  n_trays", len(state_times[item["picker_id"]])
        if verbose: print "  ", state_times[item["picker_id"]]
    return state_times

def get_state_change_counts(log_data, inc_same_states=[], verbose=False):
    # to process log_data from a single iteration
    """
    Keyword arguments:
    log_data - data read from an event log, dict
    inc_same_states - states for which same state transitions should be counted, list
    verbose - controls sysout prints, bool
    """
    state_changes = {}
    for item in log_data["sim_details"]["picker_states"]:
        picker_id = item["picker_id"]
        state_changes[picker_id] = numpy.zeros((6,6))
        curr_mode = None
        for state_info in item["state_changes"]:
            if curr_mode is None:
                # first one
                curr_mode = state_info["mode"]
            elif curr_mode == state_info["mode"]:
                # same state - ignore
                if curr_mode in inc_same_states:
                    state_changes[picker_id][curr_mode][curr_mode] += 1
            else:
                # state change
                state_changes[picker_id][curr_mode][state_info["mode"]] += 1
                curr_mode = state_info["mode"]

        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  n_state_changes:\n", state_changes[picker_id]
    return state_changes

def get_multi_iter_state_change_probs(state_changes, verbose=False):
    # to process state_changes from multiple iterations
    combined_state_changes = {}
    picker_ids = []

    for picker_id in state_changes[0]:
        picker_ids.append(picker_id)
        combined_state_changes[picker_id] = numpy.zeros((6,6))

    for item in state_changes:
        for picker_id in item:
            combined_state_changes[picker_id] += item[picker_id]

    state_change_probs = {}
    total_state_changes = {}
    for picker_id in picker_ids:
        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  combined_state_changes:\n", combined_state_changes[picker_id]
        total_state_changes[picker_id] = numpy.sum(combined_state_changes[picker_id], axis=1, dtype=numpy.float64).reshape((6,1))
        if verbose: print "  total_changes_from_each_state:\n", total_state_changes[picker_id]
        state_change_probs[picker_id] = combined_state_changes[picker_id] / total_state_changes[picker_id]
        if verbose: print "  state_change_probs:\n", state_change_probs[picker_id]

    return state_change_probs

def get_single_iter_state_change_probs(state_changes, verbose=False):
    # to process state_changes from a single iteration
    state_change_probs = {}
    total_state_changes = {}
    for picker_id in state_changes:
        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  n_state_changes:\n", state_changes[picker_id]
        total_state_changes[picker_id] = numpy.sum(state_changes[picker_id], axis=1, dtype=numpy.float64).reshape((6,1))
        if verbose: print "  total_changes_from_each_state:\n", total_state_changes[picker_id]
        state_change_probs[picker_id] = state_changes[picker_id] / total_state_changes[picker_id]
        if verbose: print "  state_change_probs:\n", state_change_probs[picker_id]

    return state_change_probs



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
    tray_picking_times = []
    state_changes = []

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

        # Time spent for pickiing each full tray
        tray_picking_times.append(get_tray_picking_times(log_data, verbose=False))

        # get state change probabilities
        state_changes.append(get_state_change_counts(log_data, inc_same_states=[], verbose=True))
#        # get state change probabilities with same state changes while in picking
#        state_changes.append(get_state_change_counts(log_data, inc_same_states=[2], verbose=True))

#==============================================================================
#     # single iter examples
#==============================================================================
    gauss_0_iter0 = get_single_iter_state_time_gauss(state_0_times[0], 0, "Idle", plot_data=True)

    state_change_probs_iter0 = get_single_iter_state_change_probs(state_changes[0], verbose=True)


#==============================================================================
#     # multi-iter examples
#==============================================================================

    # get gaussian distributions of state_0 times
    gauss_0 = get_multi_iter_state_time_gauss(state_0_times, 0, "Idle", plot_data=True, verbose=True)

    # get gaussian distributions of state_1 times
    gauss_1 = get_multi_iter_state_time_gauss(state_1_times, 1, "Transport to row node", plot_data=True, verbose=True)

    # get gaussian distributions of state_2 times
    gauss_2 = get_multi_iter_state_time_gauss(state_2_times, 2, "Picking", plot_data=True, verbose=True)

    # get gaussian distributions of state_3 times
    gauss_3 = get_multi_iter_state_time_gauss(state_3_times, 3, "Transport to storage", plot_data=True, verbose=True)

    # get gaussian distributions of state_4 times
    gauss_4 = get_multi_iter_state_time_gauss(state_4_times, 4, "Unload at storage", plot_data=True, verbose=True)

    # get gaussian distributions of tray picking times
    gauss_2_tray = get_multi_iter_state_time_gauss(tray_picking_times, 2, "Tray Picking", plot_data=True, verbose=True)

    # state change probs from multiple iterations
    state_chang_probs = get_multi_iter_state_change_probs(state_changes, verbose=True)
