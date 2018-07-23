#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import yaml
import sys
import rospy
import rasberry_des.generate_map
import rasberry_des.config_utils


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        raise Exception("Not enough arguments. Pass 'path to configuration_file used to generate gazebo_world file' as an argument.")
    else:
        print len(sys.argv)
        config_file = sys.argv[1]

    rospy.init_node("fork_map_generator", anonymous=True)

    f_handle = open(config_file, "r")
    config_data = yaml.load(f_handle)
    f_handle.close()

    keys = {}
    for i, item in enumerate(config_data):
        for key in item:
            keys[key] = i

    polytunnel_data = config_data[keys["polytunnel"]]["polytunnel"]

    _n_polytunnels = len(polytunnel_data)
    n_polytunnels = 0

    _row_length = []
    _row_spacing = []
    _n_farm_rows = []
    _row_node_dist = []
    _head_row_node_dist = []
    _x_offset = []
    y_offset = None

    # assuming second_head_lane = True
    second_head_lane = True

    for i in range(_n_polytunnels):
        if polytunnel_data[i]["include"]:
            n_polytunnels += 1
            # row_length = number of poles * distance between poles
            _row_length.append(polytunnel_data[i]["pole_nx"] * polytunnel_data[i]["pole_dx"])
            # row_spacing = distance between poles
            _row_spacing.append(polytunnel_data[i]["pole_dy"])
            # n_farm_rows = number of rows
            _n_farm_rows.append(polytunnel_data[i]["pole_ny"])
            # row_node_dist = distance between poles
            _row_node_dist.append(polytunnel_data[i]["pole_dx"])
            # x_offset = x position of first pole
            _x_offset.append(polytunnel_data[i]["pole_xoffset"])
            # TODO:
            # Distance from head lane to first row node is assumed to be half of the distance
            # between the poles
            _head_row_node_dist.append(polytunnel_data[i]["pole_dx"] / 2.0) # assumed to be same as dist between poles

            # read y_offset only once (thi is to place the next polytunnels soon after the current)
            # y_offset = y position of first row first pole - distance from first row to canopy
            # distance from first row to canopy = distance between poles
            if y_offset is None:
                y_offset = polytunnel_data[i]["pole_yoffset"] - polytunnel_data[i]["pole_dy"]

    n_farm_rows_func = "none"
    n_farm_rows = rasberry_des.config_utils.graph_param_to_poly_list("n_farm_rows", _n_farm_rows,
                                                                     n_polytunnels,
                                                                     n_farm_rows_func)

    # n_rows+1 picking rows are needed, all except first and last (in a polytunnel)
    # rows are forward and reverse. first and last are forward/reverse only
    n_topo_nav_rows = 0
    for i in range(n_polytunnels):
        n_topo_nav_rows += n_farm_rows[i] + 1

    head_row_node_dist_func = "copy"
    head_row_node_dist = rasberry_des.config_utils.graph_param_list_check("head_row_node_dist",
                                                                          _head_row_node_dist,
                                                                          n_polytunnels,
                                                                          n_farm_rows,
                                                                          n_topo_nav_rows,
                                                                          head_row_node_dist_func)

    x_offset_func = "copy"
    x_offset = rasberry_des.config_utils.graph_param_list_check("x_offset",
                                                                   _x_offset, n_polytunnels,
                                                                   n_farm_rows, n_topo_nav_rows,
                                                                   x_offset_func)

    row_node_dist_func = "copy"
    row_node_dist = rasberry_des.config_utils.graph_param_list_check("row_node_dist",
                                                                     _row_node_dist, n_polytunnels,
                                                                     n_farm_rows, n_topo_nav_rows,
                                                                     row_node_dist_func)

    row_length_func = "copy"
    row_length = rasberry_des.config_utils.graph_param_list_check("row_length",
                                                                  _row_length, n_polytunnels,
                                                                  n_farm_rows, n_topo_nav_rows,
                                                                  row_length_func)

    # row_spacing is assumed to be the spacing between the
    # farm rows or between the edge and a row, or between two rows
    # navigation rows should be at the middle of this spacing.
    row_spacing_func = "copy"
    row_spacing = rasberry_des.config_utils.graph_param_list_check("row_spacing",
                                                                   _row_spacing, n_polytunnels,
                                                                   n_farm_rows, n_topo_nav_rows,
                                                                   row_spacing_func)

    rasberry_des.generate_map.generate_fork_map( n_topo_nav_rows,
                                                head_row_node_dist, x_offset, y_offset,
                                                row_node_dist, row_length, row_spacing,
                                                second_head_lane)
