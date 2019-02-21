#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import sys
import yaml
import numpy as np


if __name__ == "__main__":
    if len(sys.argv) <2:
        print ("usage: event_log_processor.py path_to_dir_with_logs")
        exit()

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


        # Print NODE YEILD associated with each row
        n_topo_nav_rows = log_data["env_details"]["n_topo_nav_rows"]
        assert n_topo_nav_rows == len(log_data["env_details"]["row_details"])
        row_ids = []
        row_nodes = {}
        for item in log_data["env_details"]["row_details"]:
            row_ids.append(item["row_id"])
            print "row_id: %s" %(item["row_id"])
            print "  row_nodes:"
            row_nodes[item["row_id"]] = []
            for i in item["row_nodes"]:
                print " node_id:%s yield:%f" %( i["node_id"], i["yield"])
                row_nodes[item["row_id"]].append(i["node_id"])
                row_nodes[item["row_id"]].append(i["yield"])



        # Print allocated rows of each picker
        n_pickers = log_data["sim_details"]["n_pickers"]
        assert n_pickers == len(log_data["sim_details"]["picker_states"])
        picker_ids = []
        allocated_rows = {}
        for item in log_data["sim_details"]["picker_states"]:
            picker_ids.append(item["picker_id"])
            print "picker_id: %s" %(item["picker_id"])
            print "  allocated_rows:"
            allocated_rows[item["picker_id"]] = []
            count = 1
            for i in item["allocated_rows"]:
                print "    %d: %s" %(count, i["row_id"])
                allocated_rows[item["picker_id"]].append(i["row_id"])
                count += 1 
 

        # Time spent in each row for all rows in each picker case
        n_pickers = log_data["sim_details"]["n_pickers"]
        assert n_pickers == len(log_data["sim_details"]["picker_states"])
        picker_ids = []
        allocated_rows = {}
        row_ids = []
        for item in log_data["sim_details"]["picker_states"]:
            picker_ids.append(item["picker_id"])
            print "picker_id: %s" %(item["picker_id"])
            print "  allocated_rows:"
            allocated_rows[item["picker_id"]] = []
            count = 1
            for alloc_rows in item["allocated_rows"]:
                print " %d: %s: Expected_duration_each_row: %d" %(count, alloc_rows["row_id"], alloc_rows["completion_time"] - alloc_rows["allocation_time"])                
                allocated_rows[item["picker_id"]].append(alloc_rows["row_id"])
                allocated_rows[item["picker_id"]].append(alloc_rows["completion_time"] - alloc_rows["allocation_time"])
                count +=1
                
                
             
        # Get all state_0 times for all pickers
        state_0_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_0_times[item["picker_id"]] = []
            state_0_start = None
            state_0_finish = None
            MEAN = 0
            SIGMA = 0
            count = 0
            for state in item["state_changes"]:
                if state["mode"] == 0:
                    if state_0_start is None:
                        state_0_start = state["time"]
                    else:
                        pass
                else:
                    if state_0_start is None:
                        pass
                    else:
                        state_0_finish = state["time"]
                        state_0_times[item["picker_id"]].append(state_0_finish - state_0_start)
                        count += 1
                        MEAN = np.mean(state_0_times[item["picker_id"]])  # /len(state_3_times[item["picker_id"]])
                        SIGMA = np.std(state_0_times[item["picker_id"]])
                        state_0_start = None
                        state_0_finish = None
        print "--MODE:0-TRANSPORTING TO STORAGE-"        
        print state_0_times, "%s %d %s %d %s %d" %('TRANSITION_COUNT=', count, 'MEAN=', MEAN, 'SIGMA=', SIGMA)
        
        # Get all state_1 times for all pickers
        state_1_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_1_times[item["picker_id"]] = []
            state_1_start = None
            state_1_finish = None
            MEAN = 0
            SIGMA = 0
            count = 0
            for state in item["state_changes"]:
                if state["mode"] == 1:
                    if state_1_start is None:
                        state_1_start = state["time"]
                    else:
                        pass
                else:
                    if state_1_start is None:
                        pass
                    else:
                        state_1_finish = state["time"]
                        state_1_times[item["picker_id"]].append(state_1_finish - state_1_start)
                        count +=1
                        MEAN = np.mean(state_1_times[item["picker_id"]])  # /len(state_3_times[item["picker_id"]])
                        SIGMA = np.std(state_1_times[item["picker_id"]])
                        state_1_start = None
                        state_1_finish = None
        print "--MODE:1-TRANSPORTING TO STORAGE-"        
        print state_1_times, "%s %d %s %d %s %d" %('TRANSITION_COUNT=', count, 'MEAN=', MEAN, 'SIGMA=', SIGMA)
        
        # Get all state_2 times for all pickers
        state_2_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_2_times[item["picker_id"]] = []
            state_2_start = None
            state_2_finish = None
            MEAN = 0
            SIGMA = 0
            count = 0
            for state in item["state_changes"]:
                if state["mode"] == 2:
                    if state_2_start is None:
                        state_2_start = state["time"]
                    else:
                        pass
                else:
                    if state_2_start is None:
                        pass
                    else:
                        state_2_finish = state["time"]
                        state_2_times[item["picker_id"]].append(state_2_finish - state_2_start)
                        count += 1
                        MEAN = np.mean(state_2_times[item["picker_id"]])  # /len(state_3_times[item["picker_id"]])
                        SIGMA = np.std(state_2_times[item["picker_id"]])
                        state_2_start = None
                        state_2_finish = None
        print "--MODE:2-TRANSPORTING TO STORAGE-"        
        print state_2_times, "%s %d %s %d %s %d" %('TRANSITION_COUNT=', count, 'MEAN=', MEAN, 'SIGMA=', SIGMA)
        
        
               
        # Get all state_3 times for all pickers
        state_3_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_3_times[item["picker_id"]] = []
            state_3_start = None
            state_3_finish = None
            MEAN = 0
            SIGMA = 0
            count = 0
            for state in item["state_changes"]:
                if state["mode"] == 3:
                    if state_3_start is None:
                        state_3_start = state["time"]
                    else:
                        pass
                else:
                    if state_3_start is None:
                        pass
                    else:
                        state_3_finish = state["time"]
                        state_3_times[item["picker_id"]].append(state_3_finish - state_3_start)
                        count += 1                        
                        MEAN = np.mean(state_3_times[item["picker_id"]])  # /len(state_3_times[item["picker_id"]])
                        SIGMA = np.std(state_3_times[item["picker_id"]])
                        state_3_start = None
                        state_3_finish = None
        print "--MODE:3-TRANSPORTING TO STORAGE-"        

        print state_3_times, "%s %d %s %d %s %d" %('TRANSITION_COUNT=',count, 'MEAN=', MEAN, 'SIGMA=', SIGMA)
        
        # Get all state_4 times for all pickers
        state_4_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_4_times[item["picker_id"]] = []
            state_4_start = None
            state_4_finish = None
            MEAN = 0
            SIGMA = 0
            count = 0
            for state in item["state_changes"]:
                if state["mode"] == 4:
                    if state_4_start is None:
                        state_4_start = state["time"]
                    else:
                        pass
                else:
                    if state_4_start is None:
                        pass
                    else:
                        state_4_finish = state["time"]
                        state_4_times[item["picker_id"]].append(state_4_finish - state_4_start)
                        count += 1 
                        MEAN = np.mean(state_4_times[item["picker_id"]])  # /len(state_3_times[item["picker_id"]])
                        SIGMA = np.std(state_4_times[item["picker_id"]])
                        state_4_start = None
                        state_4_finish = None
        print "--MODE:4-TRANSPORTING TO STORAGE-"        
        print state_4_times, "%s %d %s %d %s %d" %('TRANSITION_COUNT=', count,'MEAN=', MEAN, 'SIGMA=', SIGMA)
        
        
