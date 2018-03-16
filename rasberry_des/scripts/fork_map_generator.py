#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.generate_map
import rasberry_des.config_utils


FORK_MAP_HEAD_NODE_Y = 1.0

if __name__ == "__main__":
    rospy.init_node("fork_map_generator", anonymous=True)

    config_params = rasberry_des.config_utils.get_fork_map_config_parameters()

    n_farm_rows = config_params["n_farm_rows"]
    half_rows = config_params["half_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]
    head_row_node_dist = config_params["head_row_node_dist"]
    head_node_y = config_params["head_node_y"]
    row_node_dist = config_params["row_node_dist"]
    row_length = config_params["row_length"]
    row_spacing = config_params["row_spacing"]

    rasberry_des.generate_map.generate_fork_map(n_farm_rows, half_rows, n_topo_nav_rows,
                                                head_row_node_dist, head_node_y,
                                                row_node_dist, row_length, row_spacing)
