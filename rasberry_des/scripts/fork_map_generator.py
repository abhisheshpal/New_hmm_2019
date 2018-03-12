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
