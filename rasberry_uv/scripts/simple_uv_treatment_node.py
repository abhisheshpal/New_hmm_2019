#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 01/10/2018
# ----------------------------------

import sys
import yaml


import rospy
import rasberry_uv.uv_treatment


if __name__ == "__main__":
    rospy.init_node("simple_uv_treatment_node")
    # should also read a yaml file and read the row information
    # like start and finish nodes of each row and pass it as an argument

    if len(sys.argv) < 2:
#        rospy.logerr("usage: rosrun rasberry_uv simple_uv_treatment_node.py config_file.yaml")
        raise Exception("usage: simple_uv_treatment_node.py config_file.yaml")
    else:
        config_file = sys.argv[1]

    f_handle = open(config_file, "r")
    config_data = yaml.load(f_handle)
    f_handle.close()

    treatment_rows = config_data["treatment_rows"]
    row_start_nodes = config_data["row_start_nodes"]
    row_finish_nodes = config_data["row_finish_nodes"]

    # make sure all row_ids in treatment_rows are in row_start_nodes and row_finish_nodes
    for row_id in treatment_rows:
#        row_id = str(row)
        if row_id in row_start_nodes and row_id in row_finish_nodes:
            pass
        elif row_id in row_start_nodes and not row_id in row_finish_nodes:
            err_msg = "configuration error. row-%d missing in row_finish_nodes" %(row_id)
            rospy.logerr(err_msg)
        elif not row_id in row_start_nodes and row_id in row_finish_nodes:
            err_msg = "configuration error. row-%d missing in row_start_nodes" %(row_id)
            rospy.logerr(err_msg)
        else:
            err_msg = "configuration error. row-%d missing in both row_start_nodes and row_finish_nodes" %(row_id)
            rospy.logerr(err_msg)

    uv_treat = rasberry_uv.uv_treatment.UVTreatment(treatment_rows=treatment_rows,
                                                    row_start_nodes=row_start_nodes,
                                                    row_finish_nodes=row_finish_nodes)
    rospy.on_shutdown(uv_treat.on_shutdown)
    first = True

    while not rospy.is_shutdown():
        # TODO: Make an elegant trigger to start the uv treatment
        if first:
            raw_input("Press ENTER to START the UV treatment")
        else:
            raw_input("Press ENTER to REDO the UV treatment")

        status = uv_treat.run()
        if not status:
            break
        first = False
