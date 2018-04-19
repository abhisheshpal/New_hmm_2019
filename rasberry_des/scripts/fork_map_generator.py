#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.generate_map
import rasberry_des.config_utils


if __name__ == "__main__":
    rospy.init_node("fork_map_generator", anonymous=True)

    config_params = rasberry_des.config_utils.get_fork_map_config_parameters()

    n_farm_rows = config_params["n_farm_rows"]
    half_rows = config_params["half_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]
    head_row_node_dist = config_params["head_row_node_dist"]
    head_node_x = config_params["head_node_x"]
    row_node_dist = config_params["row_node_dist"]
    row_length = config_params["row_length"]
    row_spacing = config_params["row_spacing"]
    if "dist_to_cold_storage" in config_params:
        dist_to_cold_storage = config_params["dist_to_cold_storage"]
        rasberry_des.generate_map.generate_fork_map(n_farm_rows, half_rows, n_topo_nav_rows,
                                                    head_row_node_dist, head_node_x,
                                                    row_node_dist, row_length, row_spacing,
                                                    dist_to_cold_storage)
    else:
        rasberry_des.generate_map.generate_fork_map(n_farm_rows, half_rows, n_topo_nav_rows,
                                                    head_row_node_dist, head_node_x,
                                                    row_node_dist, row_length, row_spacing)
