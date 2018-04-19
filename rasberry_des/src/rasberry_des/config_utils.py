#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import random
import rospy


def check_fork_map_config():
    """check whether all config params required for fork_map generation are
    available"""
    # try to get the configuration parameters required for fork_map generation
    # and if any required parameter is not available throw an error
    ns = rospy.get_namespace()
    missing_params = []
    if not rospy.has_param(ns + "rasberry_des_config/n_farm_rows"):
        missing_params.append("rasberry_des_config/n_farm_rows")
    if not rospy.has_param(ns + "rasberry_des_config/half_rows"):
        missing_params.append("rasberry_des_config/half_rows")
    if not rospy.has_param(ns + "rasberry_des_config/head_row_node_dist"):
        missing_params.append("rasberry_des_config/head_row_node_dist")
    if not rospy.has_param(ns + "rasberry_des_config/head_node_x"):
        missing_params.append("rasberry_des_config/head_node_x")
    if not rospy.has_param(ns + "rasberry_des_config/row_node_dist"):
        missing_params.append("rasberry_des_config/row_node_dist")
    if not rospy.has_param(ns + "rasberry_des_config/row_length"):
        missing_params.append("rasberry_des_config/row_length")
    if not rospy.has_param(ns + "rasberry_des_config/row_spacing"):
        missing_params.append("rasberry_des_config/row_spacing")

    return missing_params

def check_des_config():
    """"Check whether all parameters required for the DES are available"""
    # TODO: try to get the configuration parameters required for the des
    # and if any required parameter is not available throw an error.
    # some parameters might be possible to obtain from the topoplogical map
    missing_params = check_fork_map_config()
    ns = rospy.get_namespace()
    if not rospy.has_param(ns + "rasberry_des_config/n_pickers"):
        missing_params.append("rasberry_des_config/n_pickers")
    if not rospy.has_param(ns + "rasberry_des_config/picking_rate"):
        missing_params.append("rasberry_des_config/picking_rate")
    if not rospy.has_param(ns + "rasberry_des_config/picker_transportation_rate"):
        missing_params.append("rasberry_des_config/picker_transportation_rate")
    if not rospy.has_param(ns + "rasberry_des_config/picker_max_n_trays"):
        missing_params.append("rasberry_des_config/picker_max_n_trays")
    if not rospy.has_param(ns + "rasberry_des_config/picker_unloading_time"):
        missing_params.append("rasberry_des_config/picker_unloading_time")
    if not rospy.has_param(ns + "rasberry_des_config/tray_capacity"):
        missing_params.append("rasberry_des_config/tray_capacity")
    if not rospy.has_param(ns + "rasberry_des_config/yield_per_node"):
        missing_params.append("rasberry_des_config/yield_per_node")
    if not rospy.has_param(ns + "rasberry_des_config/n_local_storages"):
        missing_params.append(ns + "rasberry_des_config/n_local_storages")
    if not rospy.has_param(ns + "rasberry_des_config/n_robots"):
        missing_params.append(ns + "rasberry_des_config/n_robots")
    if not rospy.has_param(ns + "rasberry_des_config/robot_transportation_rate"):
        missing_params.append(ns + "rasberry_des_config/robot_transportation_rate")
    if not rospy.has_param(ns + "rasberry_des_config/robot_max_n_trays"):
        missing_params.append(ns + "rasberry_des_config/robot_max_n_trays")
    if not rospy.has_param(ns + "rasberry_des_config/robot_unloading_time"):
        missing_params.append(ns + "rasberry_des_config/robot_unloading_time")

    return missing_params

def param_list_to_dict(list_name, list_object, keys):
        ideal_len = len(keys)
        if list_object.__class__ == list:
            if len(list_object) == ideal_len:
                dict_object = {keys[i]:list_object[i] for i in range(ideal_len)}
                return dict_object
            else:
                rospy.ROSException("%s must be a list of size %d" %(list_name, ideal_len))
        else:
            rospy.ROSException("%s must be a list of size %d" %(list_name, ideal_len))

def des_param_list_check(param, list_object, ideal_list_len, func="copy"):
    if (list_object.__class__ == list):
        if len(list_object) == ideal_list_len:
            return list_object
        else:
            new_list = []
            if func == "copy":
                # copy single value for ideal_list_len
                if len(list_object) < 1:
                    rospy.ROSException("%s must either be an array of size %d, or %d for func='copy'" %(param, ideal_list_len, 1))
                for i in range(ideal_list_len):
                    new_list.append(list_object[0])
            elif func == "gauss":
                if len(list_object) < 1:
                    rospy.ROSException("%s must either be an array of size %d, or %d for func='gauss'" %(param, ideal_list_len, 2))
                for i in range(ideal_list_len):
                    new_list.append(random.gauss(list_object[0], list_object[1]))
            else:
                rospy.ROSException("%s must either be an array of size %d, or func must be either 'copy' or 'gauss'" %(param, ideal_list_len))
            return new_list
    else:
        rospy.ROSException("%s must either be an array" %(param))

def get_fork_map_config_parameters():
    """get_fork_map_config_parameters: get the configuration parameters requried for
    making a fork map from the rosparam server
    """
    # assume check_des_config_parameters.py is run and all parameters are present
    ns = rospy.get_namespace()
    n_farm_rows = rospy.get_param(ns + "rasberry_des_config/n_farm_rows")

    # Half or full rows
    # If half rows, n_rows-1 picking rows are needed, all forward and reverse
    # If full rows, n_rows+1 picking rows are needed, all except first and last
    # rows are forward and reverse. first and last are forward/reverse only
    half_rows = rospy.get_param(ns + "rasberry_des_config/half_rows")

    n_topo_nav_rows = n_farm_rows - 1 if half_rows else n_farm_rows + 1

    _head_row_node_dist = rospy.get_param(ns + "rasberry_des_config/head_row_node_dist")
    head_row_node_dist = des_param_list_check(ns + "rasberry_des_config/head_row_node_dist", _head_row_node_dist, n_topo_nav_rows)

    _head_node_x = rospy.get_param(ns + "rasberry_des_config/head_node_x")
    head_node_x = des_param_list_check(ns + "rasberry_des_config/head_node_x", _head_node_x, n_topo_nav_rows)

    _row_node_dist = rospy.get_param(ns + "rasberry_des_config/row_node_dist")
    row_node_dist = des_param_list_check(ns + "rasberry_des_config/row_node_dist", _row_node_dist, n_topo_nav_rows)

    # TODO: Now row_length is assumed to be for navigation rows
    # This in turn will assume picking is happening on berry rows on both sides
    # This should ideally be the length of berry rows, with picking from either
    # or both sides depending on the length of the berry rows.
    _row_length = rospy.get_param(ns + "rasberry_des_config/row_length")
    row_length = des_param_list_check(ns + "rasberry_des_config/row_length", _row_length, n_topo_nav_rows)

    # TODO: row_spacing is assumed to be the spacing between the
    # farm rows or between the edge and a row, depending on the berry rows
    # are half or full. the navigation row nodes should be at the middle of
    # this spacing. make sure it makes sense later.
    _row_spacing = rospy.get_param(ns + "rasberry_des_config/row_spacing")
    row_spacing = des_param_list_check(ns + "rasberry_des_config/row_spacing", _row_spacing, n_topo_nav_rows)

    if rospy.has_param(ns + "rasberry_des_config/dist_to_cold_storage"):
        dist_to_cold_storage = rospy.get_param(ns + "rasberry_des_config/dist_to_cold_storage")

    config_params = {}
    config_params["n_farm_rows"] = n_farm_rows
    config_params["half_rows"] = half_rows
    config_params["n_topo_nav_rows"] = n_topo_nav_rows
    config_params["head_row_node_dist"] = head_row_node_dist
    config_params["head_node_x"] = head_node_x
    config_params["row_node_dist"] = row_node_dist
    config_params["row_length"] = row_length
    config_params["row_spacing"] = row_spacing
    if rospy.has_param(ns + "rasberry_des_config/dist_to_cold_storage"):
        config_params["dist_to_cold_storage"] = dist_to_cold_storage

    return config_params

def get_des_config_parameters(map_from_db=False, n_pickers = None, n_robots=None):
    """get_des_config_parameters: Get the parameters for configuring the discrete event simulation
    from the rosparam server
    """
    ns = rospy.get_namespace()
    config_params = {}

    if not map_from_db:
        # n_rows, half_rows, n_topo_nav_rows, row_node_dist,
        # row_length, row_spacing, head_row_node_dist
        config_params = get_fork_map_config_parameters()

    else:
        # TODO: Check whether reading some parameters from db is possible
        pass

    if not config_params:
        rospy.ROSException("topo_map config parameters missing")

    n_topo_nav_rows = config_params["n_topo_nav_rows"]

    # remaining parameters

    des_env = rospy.get_param(ns + "rasberry_des_config/des_env")

    if n_pickers is None:
        n_pickers = rospy.get_param(ns + "rasberry_des_config/n_pickers")

    _picking_rate = rospy.get_param(ns + "rasberry_des_config/picking_rate")
    picking_rate = des_param_list_check(ns + "rasberry_des_config/picking_rate", _picking_rate, n_pickers, "gauss")

    _picker_transportation_rate = rospy.get_param(ns + "rasberry_des_config/picker_transportation_rate")
    picker_transportation_rate = des_param_list_check(ns + "rasberry_des_config/picker_transportation_rate", _picker_transportation_rate, n_pickers, "gauss")

    _picker_max_n_trays = rospy.get_param(ns + "rasberry_des_config/picker_max_n_trays")
    picker_max_n_trays = des_param_list_check(ns + "rasberry_des_config/picker_max_n_trays", _picker_max_n_trays, n_pickers)

    _picker_unloading_time = rospy.get_param(ns + "rasberry_des_config/picker_unloading_time")
    picker_unloading_time = des_param_list_check(ns + "rasberry_des_config/picker_unloading_time", _picker_unloading_time, n_pickers, "gauss")

    tray_capacity = rospy.get_param(ns + "rasberry_des_config/tray_capacity")

    _yield_per_node = rospy.get_param(ns + "rasberry_des_config/yield_per_node")
    yield_per_node = des_param_list_check(ns + "rasberry_des_config/yield_per_node", _yield_per_node, n_topo_nav_rows, "gauss")

    n_local_storages = rospy.get_param(ns + "rasberry_des_config/n_local_storages")

    if n_robots is None:
        n_robots = rospy.get_param(ns + "rasberry_des_config/n_robots")

    _robot_transportation_rate = rospy.get_param(ns + "rasberry_des_config/robot_transportation_rate")
    robot_transportation_rate = des_param_list_check(ns + "rasberry_des_config/robot_transportation_rate", _robot_transportation_rate, n_robots)

    _robot_max_n_trays = rospy.get_param(ns + "rasberry_des_config/robot_max_n_trays")
    robot_max_n_trays = des_param_list_check(ns + "rasberry_des_config/robot_max_n_trays", _robot_max_n_trays, n_robots)

    _robot_unloading_time = rospy.get_param(ns + "rasberry_des_config/robot_unloading_time")
    robot_unloading_time = des_param_list_check(ns + "rasberry_des_config/robot_unloading_time", _robot_unloading_time, n_robots)

    config_params["des_env"] = des_env
    config_params["n_pickers"] = n_pickers
    config_params["picking_rate"] = picking_rate
    config_params["picker_transportation_rate"] = picker_transportation_rate
    config_params["picker_max_n_trays"] = picker_max_n_trays
    config_params["picker_unloading_time"] = picker_unloading_time
    config_params["tray_capacity"] = tray_capacity
    config_params["yield_per_node"] = yield_per_node
    config_params["n_local_storages"] = n_local_storages
    config_params["n_robots"] = n_robots
    config_params["robot_transportation_rate"] = robot_transportation_rate
    config_params["robot_max_n_trays"] = robot_max_n_trays
    config_params["robot_unloading_time"] = robot_unloading_time

    return config_params
