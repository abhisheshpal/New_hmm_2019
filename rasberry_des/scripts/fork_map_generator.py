#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.generate_map
import rasberry_des.config_utils
import sys


FORK_MAP_HEAD_NODE_Y = 1.0

if __name__ == "__main__":
    rospy.init_node("fork_map_generator", anonymous=True)

    if len(sys.argv) < 2:
        ns = "/rasberry_des_config/"
    else:
        ns = sys.argv[1] if sys.argv[1][-1] == "/" else sys.argv[1] + "/"

    config_params = rasberry_des.config_utils.get_fork_map_config_parameters(ns)

    n_farm_rows = config_params[0]
    half_rows = config_params[1]
    n_topo_nav_rows = config_params[2]
    head_row_node_dist = config_params[3]
    head_node_y = config_params[4]
    row_node_dist = config_params[5]
    row_length = config_params[6]
    row_spacing = config_params[7]

    rasberry_des.generate_map.generate_fork_map(n_farm_rows, half_rows, n_topo_nav_rows,
                                                head_row_node_dist, head_node_y,
                                                row_node_dist, row_length, row_spacing)
