#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import sys
import rospy
import rasberry_des.generate_map
import rasberry_des.config_utils


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        raise Exception("Not enough arguments. Pass 'path to configuration file' as an argument.")
    else:
        config_file = sys.argv[1]

    rospy.init_node("fork_map_generator", anonymous=True)

    config_params = rasberry_des.config_utils.get_fork_map_config_parameters(config_file)

    n_topo_nav_rows = config_params["n_topo_nav_rows"]
    head_row_node_dist = config_params["head_row_node_dist"]
    x_offset = config_params["x_offset"]
    y_offset = config_params["y_offset"]
    row_node_dist = config_params["row_node_dist"]
    row_length = config_params["row_length"]
    row_spacing = config_params["row_spacing"]
    second_head_lane = config_params["second_head_lane"]

    if "dist_to_cold_storage" in config_params:
        dist_to_cold_storage = config_params["dist_to_cold_storage"]
        rasberry_des.generate_map.generate_fork_map(n_topo_nav_rows,
                                                    head_row_node_dist, x_offset, y_offset,
                                                    row_node_dist, row_length, row_spacing,
                                                    second_head_lane,
                                                    dist_to_cold_storage)
    else:
        rasberry_des.generate_map.generate_fork_map( n_topo_nav_rows,
                                                    head_row_node_dist, x_offset, y_offset,
                                                    row_node_dist, row_length, row_spacing,
                                                    second_head_lane)
