#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import os
import sys
import yaml


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

        # Example 1: print allocated rows of each picker
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
            for alloc_rows in item["allocated_rows"]:
                print "    %d: %s" %(count, alloc_rows["row_id"])
                allocated_rows[item["picker_id"]].append(alloc_rows["row_id"])
                count += 1

        # Example 2: get all state_0 times for all pickers
        state_0_times = {}
        for item in log_data["sim_details"]["picker_states"]:
            state_0_times[item["picker_id"]] = []
            state_0_start = None
            state_0_finish = None
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
                        state_0_start = None
                        state_0_finish = None
        print state_0_times
