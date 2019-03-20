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


def get_picker_ids(log_data, verbose=False):
    """get_picker_ids - gets picker_ids from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    verbose - controls sysout prints, bool

    Returns:

    picker_ids - list of picker_ids
    """
    # to process log_data from a single iteration
    picker_ids = []
    n_pickers = log_data["sim_details"]["n_pickers"]
    assert n_pickers == len(log_data["sim_details"]["picker_states"])
    for item in log_data["sim_details"]["picker_states"]:
        if verbose: print "picker_id: %s" %(item["picker_id"])
        picker_ids.append(item["picker_id"])
    return picker_ids

def get_node_yields(log_data, verbose=False):
    """get_node_yields - gets node_yields from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    verbose - controls sysout prints, bool

    Returns:

    node_yields - {node_id: yield}
    """
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
    """get_allocated_rows - gets rows allocated from each picker from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    verbose - controls sysout prints, bool

    Returns:

    allocated_rows - {picker_id: [row_ids]}
    """
    # to process log_data from a single iteration
    allocated_rows = {} # {picker_id: [allcoated_row_id_1, allcoated_row_id_2, ...]}
    n_pickers = log_data["sim_details"]["n_pickers"]
    assert n_pickers == len(log_data["sim_details"]["picker_states"])
#    allocated_rows = {}
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

#--------------ON GOING WORK-------------------------

"""
Create two functions , one for getting the difference between the X coordinates of the consecutive nodes
second function for (by using function one) getting the rates of the modes based of time taken to traverse between 
consecutive nodes. 

"""
def get_node_cord(log_data, verbose= False):

#    row_node = []
    n_topo_nav_rows = log_data["env_details"]["n_topo_nav_rows"]     
    assert n_topo_nav_rows == len(log_data["env_details"]["row_details"])
    
    x = None
    y = None
    x_new = None
    y_new = None
#    node_ids = []
    xy_diff = [] 
    count = 0
    for item in log_data["env_details"]["row_details"]:
        for rows in item["row_id"]: 
#        row_node.append(item["row_id"])
#    for rows in item["row_details"]:
#        row_node[item["row_id"]] = []
            for row_nod in rows["row_nodes"]:
    #            row_node(item["row_id"]) = node_ids.append(row_node["node_id"])
    #            row_node[item["row_id"]].append(row_node["node_id"])
              for all_nodes in row_nod["node_id"]:
                  if x and y is None:
                    x_new = x + all_nodes["X"]
                    y_new = y + all_nodes["Y"]
                    xy_diff = math.sqrt(x_new^2 - y_new^2)       
                  else:
                    xy_diff = math.sqrt((x_new-x)^2 - (y_new-y)^2)
#                    row_node[item["row_id"]["node_id"]].append(xy_diff)  
                    count += 1
                    x = None
                    y = None
                    x_new = None
                    y_new = None
                  
    return xy_diff
                
#def get_node_cord_diff(log_data, verbose=False):
#    row_node = {}
#    n_topo_nav_rows = log_data["env_details"]["n_topo_nav_rows"]     
#    assert n_topo_nav_rows == len(log_data["env_details"]["row_details"])
#    row_ids = []            # created list to store row_id 
##    row_node = {}           # created dict to store row_nodes
#    node_coordinates_X = [] # {node_id: X, Y}  # created a list to store coordinate X   
##    head_nodes = {}         # created dict to store head_nodes
#    count = 0
#    X_coord_diff = 0
#    for item in log_data["env_details"]["row_details"]:
#        row_ids.append(item["row_id"])   # append the row_ids list with row_id
#        if verbose: print "row_id: %s" %(item["row_id"])    # print row_id: []
##        if verbose: print "  row_nodes:"                    # print row_nodes:
#        row_node[item["row_id"]] = {}   # initialize row_node dict with key row_id: _value_ 
#        for node_info in item["row_nodes"]:
#            for node_ids in node_info["node_id"]:
#                node_coordinates_X = node_info["X"]
#                node_coordinates_X.append(node_info["X"])
#                row_node.append(item["node_id"])
#                X_coord_diff =  node_coordinates_X.append(node_info["X"]) - node_coordinates_X
#                count += 1
#                if verbose: print "%d node_id:%s X: %0.3f" %(count, node_info["node_id"], node_info["X"])
#                
#    return X_coord_diff


def get_mode_rate(log_data, mode, mode_str, verbose=False):
    """get_mode_rate - gets rate of a picker in a given mode when traversing between the nodes 
    
    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    mode - mode of interest
    mode_str - mode string, for printing
    verbose - controls sysout prints, bool

    Returns:

    mode_rate - {picker_id: [rate]}
    """

    mode_rate = {}
    if verbose: print "MODE:%d - %s" %(mode, mode_str)
    for item in log_data["sim_details"]["picker_states"]:
        mode_rate[item["picker_id"]] = []
        mode_start = None
        mean = 0
        sigma = 0
        count = 0
        rate = 0
        get_node_id = 0
        for mode_info in item["state_changes"]:
            if mode_info["mode"] == mode:
                if mode_start is None:
                    mode_start = mode_info["time"]
                    get_node_id = get_node_cord(log_data, verbose=False)
                    rate = get_node_id / mode_start
          
                else:
                    mode_rate[item["picker_id"]].append(rate)
                    count += 1
                    mode_start = None
                    get_node_id = 0

        mean = numpy.mean(mode_rate[item["picker_id"]])
        sigma = numpy.std(mode_rate[item["picker_id"]])

        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  transition count: %d, mean_time: %0.3f, std: %0.3f" %(count, mean, sigma)
        if verbose: print "  ", mode_rate[item["picker_id"]]

    return mode_rate

#--------------------------------------------------------

def get_time_spent_in_rows(log_data, verbose=False):
    """get_time_spent_in_rows - gets rough measures of time each picker spent
    in rows from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    verbose - controls sysout prints, bool

    Returns:

    time_spent_in_rows - {picker_id: {row_id: time}}
    """
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


def get_mode_times(log_data, mode, mode_str, verbose=False):
    """get_mode_times - gets times a picker spent in a given mode from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    mode - mode of interest
    mode_str - mode string, for printing
    verbose - controls sysout prints, bool

    Returns:

    mode_times - {picker_id: [times]}
    """

    # to process log_data from a single iteration
    mode_times = {}
    if verbose: print "MODE:%d - %s" %(mode, mode_str)
    for item in log_data["sim_details"]["picker_states"]:
        mode_times[item["picker_id"]] = []
        mode_start = None
        mode_finish = None
        mean = 0
        sigma = 0
        count = 0
        for mode_info in item["state_changes"]:
            if mode_info["mode"] == mode:
                if mode_start is None:
                    mode_start = mode_info["time"]
                else:
                    pass
            else:
                if mode_start is None:
                    pass
                else:
                    mode_finish = mode_info["time"]
                    mode_times[item["picker_id"]].append(mode_finish - mode_start)
                    count += 1
                    mode_start = None
                    mode_finish = None


        mean = numpy.mean(mode_times[item["picker_id"]])
        sigma = numpy.std(mode_times[item["picker_id"]])

        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  transition count: %d, mean_time: %0.3f, std: %0.3f" %(count, mean, sigma)
        if verbose: print "  ", mode_times[item["picker_id"]]

    return mode_times


#-------------------------------------------

def get_mode_nodes(log_data, mode, mode_str, verbose=False):
    """get_mode_nodes - gets nodes a picker was in, in a given mode, from a log_data file

    Keyword arguments:

    log_data - data dict loaded from a yaml log file
    mode - mode of interest
    mode_str - mode string (for printing)
    verbose - controls sysout prints, bool

    Returns:

    mode_nodes - {picker_id: [nodes]}
    """
    # to process log_data from a single iteration
    mode_nodes = {}
    if verbose: print "MODE:%d - %s" %(mode, mode_str)
    for item in log_data["sim_details"]["picker_states"]:
        mode_nodes[item["picker_id"]] = []
        count = 0
        for mode_info in item["state_changes"]:
            if mode_info["mode"] == mode:
                mode_nodes[item["picker_id"]].append(mode_info["node"])
                count += 1
        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  ", mode_nodes[item["picker_id"]]

    return mode_nodes

#----------------------------------------------


def get_multi_iter_mode_time_gauss(mode_times, mode, mode_str, plot_data=False, verbose=False):
    """get_multi_iter_mode_time_gauss - gets the mean and std of the gaussian
     for all pickers, from mode_times from multiple iterations

    Keyword arguments:

    mode_times - list of mode_times from log files obtained from different iterations
    mode - mode of interest
    mode_str - mode string (for printing)
    plot_data - enable/disable plotting
    verbose - controls sysout prints, bool

    Returns:

    gauss_distributions - {picker_id: {mean: value, std: value}}
    """
    gauss_distributions = {} # {picker: {mean:value, sigma:value}}
    # mode_times = [{picker_01:[...], picker_02:[...]}]
    # assuming the number of pickers and picker ids are same in all iters
    picker_ids = mode_times[0].keys()
    all_times = {}
    for picker_id in picker_ids:
        all_times[picker_id] = []
        gauss_distributions[picker_id] = {}

    # get all times from each picker
    for item in mode_times:
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
            ax1.set_title("%d - %s" %(mode, mode_str))

            ax2 = fig.add_subplot(212)
            x = numpy.arange(min_time, max_time, .01)
            ax2.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Probability')
            ax2.set_title('Prio Probability distribution-' "%d - %s" %(mode, mode_str))
            fig.savefig("multi_iter distribution - mode: %d - %s.png" %(mode, mode_str))
            fig.subplots_adjust(left=0.125, bottom=None, right=0.9, top=0.9, wspace=0.5, hspace=0.2)
            matplotlib.pyplot.close(fig)

    return gauss_distributions

def get_single_iter_mode_time_gauss(mode_times, mode, mode_str, plot_data=False):
    """get_multi_iter_mode_time_gauss - gets the mean and std of the gaussian
     for all pickers, from mode_times from single iteration

    Keyword arguments:

    mode_times - mode_times from log files obtained from different iterations
    mode - mode of interest
    mode_str - mode string (for printing)
    plot_data - enable/disable plotting
    verbose - controls sysout prints, bool

    Returns:

    gauss_distributions - {picker_id: {mean: value, std: value}}
    """
    # to process log_data from a single iteration
    gauss_distributions = {} # {picker: {mean:value, sigma:value}}
    # mode_times = {picker_01:[...], picker_02:[...]}
    picker_ids = mode_times.keys()

    for picker_id in picker_ids:
        gauss_distributions[picker_id] = {}

    for picker_id in picker_ids:
        mean = numpy.mean(mode_times[picker_id])
        sigma = numpy.std(mode_times[picker_id])

        gauss_distributions[picker_id]["mean"] = mean
        gauss_distributions[picker_id]["sigma"] = sigma

        if plot_data:
            min_time = numpy.min(mode_times[picker_id])
            max_time = numpy.max(mode_times[picker_id])
            fig = matplotlib.pyplot.figure()
            ax1 = fig.add_subplot(211)
            n, bins, patches = ax1.hist(mode_times[picker_id], bins=int(math.ceil((max_time-min_time)/(0.01*(max_time-min_time)))), range=(min_time, max_time))
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('No. of instances')
            ax1.set_title("%d - %s" %(mode, mode_str))

            ax2 = fig.add_subplot(212)
            x = numpy.arange(min_time, max_time, .001)
            ax2.plot(x, 1/(sigma * numpy.sqrt(2 * numpy.pi)) * numpy.exp( - (x - mean)**2 / (2 * sigma**2) ), linewidth=2, color='r')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Probability')
            ax2.set_title('Prio Probability distribution-' "%d - %s" %(mode, mode_str))
            fig.savefig("single_iter distribution - mode: %d - %s.png" %(mode, mode_str))
            fig.subplots_adjust(left=0.125, bottom=None, right=0.9, top=0.9, wspace=0.5, hspace=0.2)
            matplotlib.pyplot.close(fig)

    return gauss_distributions

def isclose(a, b, rel_tol=1e-06, abs_tol=0.0):
    """isclose - to check two floats a and b are close (nearly equal)

    Keyword arguments:

    a - value 1
    b - value 2

    Returns:

    isclose - True / False
    """
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def get_tray_picking_times(log_data, verbose=False):
    """get_tray_picking_times - gets tray picking times of all pickers from
    log_data from a single iteration. As a tray could be filled from more than
    one row, depending on where it started, it is different from mode-2 times.

    Keyword arguments:

    log_data - log data loaded from an event log yaml file
    verbose - controls sysout prints, bool

    Returns:

    mode_times - {picker_id: [tray_picking_times]}
    """

    # to process log_data from a single iteration
    mode_times = {}
    if verbose: print "Picking Times per Tray"
    for item in log_data["sim_details"]["picker_states"]:
        mode_times[item["picker_id"]] = []
        mode_start = None
        mode_finish = None
        mode_delta = 0.
        mean = 0
        sigma = 0
        count = 0
        tray_started = False
        prev_mode = None
        for mode_info in item["state_changes"]:
            if tray_started:
                if mode_info["mode"] == 0:
                    mode_finish = mode_info["time"]
                    mode_delta += (mode_finish - mode_start)
                    prev_mode = 0
                elif mode_info["mode"] == 1:
                    prev_mode = 1
                elif mode_info["mode"] == 2:
                    if prev_mode != 2:
                        mode_start = mode_info["time"]
                    prev_mode = 2
                elif mode_info["mode"] == 3:
                    mode_finish = mode_info["time"]
                    mode_delta += (mode_finish - mode_start)
                    mode_times[item["picker_id"]].append(mode_delta)
                    count += 1
                    mode_start = None
                    mode_finish = None
                    mode_delta = 0.
                    tray_started = False
                    prev_mode = 3
                elif mode_info["mode"] == 4:
                    prev_mode = 4
                elif mode_info["mode"] == 5:
                    mode_finish = mode_info["time"]
                    mode_delta += (mode_finish - mode_start)
                    mode_times[item["picker_id"]].append(mode_delta)
                    count += 1
                    mode_start = None
                    mode_finish = None
                    mode_delta = 0.
                    tray_started = False
                    prev_mode = 5
                elif mode_info["mode"] == 6:
                    prev_mode = 6
            else:
                if mode_info["mode"] == 0:
                    prev_mode = 0
                elif mode_info["mode"] == 1:
                    prev_mode = 1
                elif mode_info["mode"] == 2:
                    mode_start = mode_info["time"]
                    tray_started = True
                    prev_mode = 2
                elif mode_info["mode"] == 3:
                    prev_mode = 3
                elif mode_info["mode"] == 4:
                    prev_mode = 4
                elif mode_info["mode"] == 5:
                    prev_mode = 5
                elif mode_info["mode"] == 6:
                    prev_mode = 6

        mean = numpy.mean(mode_times[item["picker_id"]])
        sigma = numpy.std(mode_times[item["picker_id"]])
        if verbose: print "picker_id: %s" %(item["picker_id"])
        if verbose: print "  transition count: %d, mean_time: %0.3f, std: %0.3f" %(count, mean, sigma)
        if verbose: print "  n_trays", len(mode_times[item["picker_id"]])
        if verbose: print "  ", mode_times[item["picker_id"]]
    return mode_times

def get_mode_change_counts(log_data, inc_same_modes=[], verbose=False):
    """get_mode_change_counts - get number of times a picker changed from one mode to another

    Keyword arguments:

    log_data - data read from an event log, dict
    inc_same_modes - modes for which same mode transitions should be counted, list
    verbose - controls sysout prints, bool

    Returns:

    mode_changes - {picker_id: np.array(n_modes x n_modes)}
                   element [from_mode][to_mode]  of array gives the number of
                   times the picker changed mode from from_mode to to_mode
    """
    # to process log_data from a single iteration
    mode_changes = {}
    for item in log_data["sim_details"]["picker_states"]:
        picker_id = item["picker_id"]
        mode_changes[picker_id] = numpy.zeros((6,6))
        curr_mode = None
        for mode_info in item["state_changes"]:
            if curr_mode is None:
                # first one
                curr_mode = mode_info["mode"]
            elif curr_mode == mode_info["mode"]:
                # same mode - ignore
                if curr_mode in inc_same_modes:
                    mode_changes[picker_id][curr_mode][curr_mode] += 1
            else:
                # mode change
                mode_changes[picker_id][curr_mode][mode_info["mode"]] += 1
                curr_mode = mode_info["mode"]

        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  n_mode_changes:\n", mode_changes[picker_id]
    return mode_changes

def get_multi_iter_mode_change_probs(mode_changes, verbose=False):
    """get_multi_iter_mode_change_probs - get the probability of changes
    for a picker from from one mode to another, from multiple iterations

    Keyword arguments:

    mode_changes - list of mode change counts from multiple iterations
    verbose - controls sysout prints, bool

    Returns:

    mode_change_probs - {picker_id: np.array(n_modes x n_modes)}
                   element [from_mode][to_mode]  of array gives the probability
                   of the picker changing mode from from_mode to to_mode
    """
    # to process mode_changes from multiple iterations
    combined_mode_changes = {}
    picker_ids = []

    for picker_id in mode_changes[0]:
        picker_ids.append(picker_id)
        combined_mode_changes[picker_id] = numpy.zeros((6,6))

    for item in mode_changes:
        for picker_id in item:
            combined_mode_changes[picker_id] += item[picker_id]

    mode_change_probs = {}
    total_mode_changes = {}
    for picker_id in picker_ids:
        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  combined_mode_changes:\n", combined_mode_changes[picker_id]
        total_mode_changes[picker_id] = numpy.sum(combined_mode_changes[picker_id], axis=1, dtype=numpy.float64).reshape((6,1))
        if verbose: print "  total_changes_from_each_mode:\n", total_mode_changes[picker_id]
        mode_change_probs[picker_id] = combined_mode_changes[picker_id] / total_mode_changes[picker_id]
        if verbose: print "  mode_change_probs:\n", mode_change_probs[picker_id]

    return mode_change_probs

def get_single_iter_mode_change_probs(mode_changes, verbose=False):
    """get_single_iter_mode_change_probs - get the probability of changes
    for a picker from from one mode to another, from single iteration

    Keyword arguments:

    mode_changes - list of mode change counts from multiple iterations
    verbose - controls sysout prints, bool

    Returns:

    mode_change_probs - {picker_id: np.array(n_modes x n_modes)}
                   element [from_mode][to_mode]  of array gives the probability
                   of the picker changing mode from from_mode to to_mode
    """
    # to process mode_changes from a single iteration
    mode_change_probs = {}
    total_mode_changes = {}
    for picker_id in mode_changes:
        if verbose: print "picker: %s" %(picker_id)
        if verbose: print "  n_mode_changes:\n", mode_changes[picker_id]
        total_mode_changes[picker_id] = numpy.sum(mode_changes[picker_id], axis=1, dtype=numpy.float64).reshape((6,1))
        if verbose: print "  total_changes_from_each_mode:\n", total_mode_changes[picker_id]
        mode_change_probs[picker_id] = mode_changes[picker_id] / total_mode_changes[picker_id]
        if verbose: print "  mode_change_probs:\n", mode_change_probs[picker_id]

    return mode_change_probs

def get_log_data(f_name, verbose=False):
    """get_log_data - get log data from event log file (yaml format)

    Keyword arguments:

    f_name - name of the log file
    verbose - controls sysout prints, bool

    Returns:

    log_data - data loaded from the event log file
    """
    f_handle = open(f_name, "r")
    try:
        log_data = yaml.load(f_handle)
    except:
        if verbose: print "Error loading file %s as a YAML file\nProceeding to next file" %(f_name)
        log_data = None
        f_handle.close()
    else:
        f_handle.close()
    return log_data

if __name__ == "__main__":
    if len(sys.argv) <2:
        print ("usage: event_log_processor.py path_to_dir_with_logs")
        exit()

    n_trials = 0
    mode_0_times = []
    mode_1_times = []
    mode_2_times = []
    mode_3_times = []
    mode_4_times = []
    tray_picking_times = []
    mode_changes = []

    logs_dir = os.path.abspath(sys.argv[1])
    for f_name in os.listdir(logs_dir):
        if not (os.path.isfile(logs_dir+"/"+f_name) and f_name.startswith("M") and f_name.endswith(".yaml")):
            continue

        print f_name
        log_data = get_log_data(logs_dir+"/"+f_name, verbose=False)
        if log_data is None:
            print "Error loading file %s as a YAML file\nProceeding to next file" %(f_name)
            continue

        n_trials += 1


        # node yield associated with each row
        node_yields = get_node_yields(log_data, verbose=False)

        # allocated rows of each picker
        allocated_rows = get_allocated_rows(log_data, verbose=False)

        # Time spent in each row for all rows in each picker case
        time_spent_in_row = get_time_spent_in_rows(log_data, verbose=False)

        # Get all mode_0 times for all pickers
        mode_0_times.append(get_mode_times(log_data, 0, "Idle", verbose=True))

        # Get all mode_1 times for all pickers
        mode_1_times.append(get_mode_times(log_data, 1, "Transport to row node", verbose=True))

        # Get all mode_2 times for all pickers
        mode_2_times.append(get_mode_times(log_data, 2, "Picking", verbose=True))

        # Get all mode_3 times for all pickers
        mode_3_times.append(get_mode_times(log_data, 3, "Transport to storage", verbose=True))

        # Get all mode_4 times for all pickers
        mode_4_times.append(get_mode_times(log_data, 4, "Unload at storage", verbose=True))

        # Time spent for pickiing each full tray
        tray_picking_times.append(get_tray_picking_times(log_data, verbose=False))

        # get mode change probabilities
        mode_changes.append(get_mode_change_counts(log_data, inc_same_modes=[], verbose=True))
        # get mode change probabilities with same mode changes while in picking
        mode_changes.append(get_mode_change_counts(log_data, inc_same_modes=[2], verbose=True))

#        # Get all mode_0 node changes for all pickers
#        mode_0_node_change = get_mode_nodes(log_data, 0, "Idle", verbose=True)
#        #print mode_0_node_change
#        # Get all mode_1 node changes for all pickers
#        mode_1_node_change = get_mode_nodes(log_data, 1, "Transport to row node", verbose=True)
#        #print mode_1_node_change
#        # Get all mode_2 node changes for all pickers
#        mode_2_node_change = get_mode_nodes(log_data, 2, "Picking", verbose=True)
#        #print mode_2_node_change        
#        # Get all mode_3 node changes for all pickers
#        mode_3_node_change = get_mode_nodes(log_data, 3, "Transport to storage", verbose=True)
#        #print mode_3_node_change        
#        # Get all mode_4 node changes for all pickers
#        mode_4_node_change = get_mode_nodes(log_data, 4, "Unload at storage", verbose=True)
#        #print mode_4_node_change

        get_rates_node = get_mode_rate(log_data, 0, "Idle", verbose=True)
        print "here: %s" %(get_rates_node)




#==============================================================================
#     # single iter examples
#==============================================================================
    gauss_0_iter0 = get_single_iter_mode_time_gauss(mode_0_times[0], 0, "Idle", plot_data=True)
#
    mode_change_probs_iter0 = get_single_iter_mode_change_probs(mode_changes[0], verbose=True)


#==============================================================================
#     # multi-iter examples
#==============================================================================

    # get gaussian distributions of mode_0 times
    gauss_0 = get_multi_iter_mode_time_gauss(mode_0_times, 0, "Idle", plot_data=True, verbose=True)

    # get gaussian distributions of mode_1 times
    gauss_1 = get_multi_iter_mode_time_gauss(mode_1_times, 1, "Transport to row node", plot_data=True, verbose=True)

    # get gaussian distributions of mode_2 times
    gauss_2 = get_multi_iter_mode_time_gauss(mode_2_times, 2, "Picking", plot_data=True, verbose=True)

    # get gaussian distributions of mode_3 times
    gauss_3 = get_multi_iter_mode_time_gauss(mode_3_times, 3, "Transport to storage", plot_data=True, verbose=True)

    # get gaussian distributions of mode_4 times
    gauss_4 = get_multi_iter_mode_time_gauss(mode_4_times, 4, "Unload at storage", plot_data=True, verbose=True)

    # get gaussian distributions of tray picking times

    gauss_2_tray = get_multi_iter_mode_time_gauss(tray_picking_times, 2, "Tray Picking", plot_data=True, verbose=True)

    # mode change probs from multiple iterations
    mode_change_probs = get_multi_iter_mode_change_probs(mode_changes, verbose=True)