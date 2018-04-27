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
    if not rospy.has_param(ns + "rasberry_des_config/n_polytunnels"):
        missing_params.append("rasberry_des_config/n_polytunnels")
    if not rospy.has_param(ns + "rasberry_des_config/n_farm_rows"):
        missing_params.append("rasberry_des_config/n_farm_rows")
    if not rospy.has_param(ns + "rasberry_des_config/n_farm_rows_func"):
        missing_params.append("rasberry_des_config/n_farm_rows_func")
    if not rospy.has_param(ns + "rasberry_des_config/head_row_node_dist"):
        missing_params.append("rasberry_des_config/head_row_node_dist")
    if not rospy.has_param(ns + "rasberry_des_config/head_row_node_dist_func"):
        missing_params.append("rasberry_des_config/head_row_node_dist_func")
    if not rospy.has_param(ns + "rasberry_des_config/head_node_x"):
        missing_params.append("rasberry_des_config/head_node_x")
    if not rospy.has_param(ns + "rasberry_des_config/head_node_x_func"):
        missing_params.append("rasberry_des_config/head_node_x_func")
    if not rospy.has_param(ns + "rasberry_des_config/row_node_dist"):
        missing_params.append("rasberry_des_config/row_node_dist")
    if not rospy.has_param(ns + "rasberry_des_config/row_node_dist_func"):
        missing_params.append("rasberry_des_config/row_node_dist_func")
    if not rospy.has_param(ns + "rasberry_des_config/row_length"):
        missing_params.append("rasberry_des_config/row_length")
    if not rospy.has_param(ns + "rasberry_des_config/row_length_func"):
        missing_params.append("rasberry_des_config/row_length_func")
    if not rospy.has_param(ns + "rasberry_des_config/row_spacing"):
        missing_params.append("rasberry_des_config/row_spacing")
    if not rospy.has_param(ns + "rasberry_des_config/row_spacing_func"):
        missing_params.append("rasberry_des_config/row_spacing_func")

    return missing_params

def check_des_config():
    """"Check whether all parameters required for the DES are available"""
    # TODO: some parameters might be possible to obtain from the topoplogical map
    missing_params = check_fork_map_config()
    ns = rospy.get_namespace()
    if not rospy.has_param(ns + "rasberry_des_config/n_pickers"):
        missing_params.append("rasberry_des_config/n_pickers")
    if not rospy.has_param(ns + "rasberry_des_config/picker_picking_rate"):
        missing_params.append("rasberry_des_config/picker_picking_rate")
    if not rospy.has_param(ns + "rasberry_des_config/picker_picking_rate_func"):
        missing_params.append("rasberry_des_config/picker_picking_rate_func")
    if not rospy.has_param(ns + "rasberry_des_config/picker_transportation_rate"):
        missing_params.append("rasberry_des_config/picker_transportation_rate")
    if not rospy.has_param(ns + "rasberry_des_config/picker_transportation_rate_func"):
        missing_params.append("rasberry_des_config/picker_transportation_rate_func")
    if not rospy.has_param(ns + "rasberry_des_config/picker_max_n_trays"):
        missing_params.append("rasberry_des_config/picker_max_n_trays")
    if not rospy.has_param(ns + "rasberry_des_config/picker_max_n_trays_func"):
        missing_params.append("rasberry_des_config/picker_max_n_trays_func")
    if not rospy.has_param(ns + "rasberry_des_config/picker_unloading_time"):
        missing_params.append("rasberry_des_config/picker_unloading_time")
    if not rospy.has_param(ns + "rasberry_des_config/picker_unloading_time_func"):
        missing_params.append("rasberry_des_config/picker_unloading_time_func")
    if not rospy.has_param(ns + "rasberry_des_config/tray_capacity"):
        missing_params.append("rasberry_des_config/tray_capacity")
    if not rospy.has_param(ns + "rasberry_des_config/yield_per_node"):
        missing_params.append("rasberry_des_config/yield_per_node")
    if not rospy.has_param(ns + "rasberry_des_config/yield_per_node_func"):
        missing_params.append("rasberry_des_config/yield_per_node_func")
    if not rospy.has_param(ns + "rasberry_des_config/n_local_storages"):
        missing_params.append(ns + "rasberry_des_config/n_local_storages")
    if not rospy.has_param(ns + "rasberry_des_config/n_robots"):
        missing_params.append(ns + "rasberry_des_config/n_robots")
    # if n_robots is zero, these parameters may not be needed. But considered as needed as
    # the simulation may have to be run in a loop
    if not rospy.has_param(ns + "rasberry_des_config/robot_transportation_rate"):
        missing_params.append(ns + "rasberry_des_config/robot_transportation_rate")
    if not rospy.has_param(ns + "rasberry_des_config/robot_max_n_trays"):
        missing_params.append(ns + "rasberry_des_config/robot_max_n_trays")
    if not rospy.has_param(ns + "rasberry_des_config/robot_max_n_trays_func"):
        missing_params.append(ns + "rasberry_des_config/robot_max_n_trays_func")
    if not rospy.has_param(ns + "rasberry_des_config/robot_unloading_time"):
        missing_params.append(ns + "rasberry_des_config/robot_unloading_time")

    return missing_params

def param_list_to_dict(list_name, list_object, keys):
    """convert a list of values (list_object) to a dict with keys (keys)

    Keyword arguments:

    list_name -- string name of the list object for exception purposes
    list_object -- list of values
    keys -- list of keys to be used in dict (len same as that of list_object)
    """
    ideal_len = len(keys)
    if list_object.__class__ != list:
        raise Exception("%s must be a list of size %d" %(list_name, ideal_len))

    if len(list_object) != ideal_len:
        raise Exception("%s must be a list of size %d" %(list_name, ideal_len))
    dict_object = {keys[i]:list_object[i] for i in range(ideal_len)}
    return dict_object

def des_param_list_check(param, list_object, ideal_list_len, func="copy"):
    """check the size of a list of values (list_object) of parameter (param) and extend to a
    list of len ideal_list_len. use func to determine how this can be done.

    Keyword arguments:

    param --string name of the parameter for exception purposes
    list_object -- list of values
    ideal_list_len -- ideal length of the list
    func -- func to be used to extend list_object to len ideal_list_len (none, copy or gauss)
    """
    if list_object.__class__ != list:
        raise Exception("%s must be a list" %(param))

    if func == "none":
        # there must be required number of values
        if len(list_object) != ideal_list_len:
            raise Exception("%s must be a list of size %d for func='none'" %(param, ideal_list_len))
        new_list = list_object

    elif func == "copy":
        # copy single value for ideal_list_len
        if len(list_object) != 1:
            raise Exception("%s must be a list of size 1 for func='copy'" %(param))
        new_list = [list_object[0] for i in range(ideal_list_len)]

    elif func == "gauss":
        if len(list_object) != 2:
            raise Exception("%s must be a list of size 2 for func='gauss'" %(param))
        new_list = [random.gauss(list_object[0], list_object[1]) for i in range(ideal_list_len)]

    else:
        raise Exception("%s must be a list of (size %d & func='none'), (size 1 & func='copy') or (size 2 & func='gauss')" %(param, ideal_list_len))
    return new_list

def graph_param_list_check(param, list_object, n_polytunnels, n_farm_rows,
                           n_topo_nav_rows, func):
    """function to extend graph parameters to a list of size n_topo_nav_rows from
    one or n_polytunnels values
    """
    # head_row_dose_dist, head_node_x, row_node_dist, row_length, row_spacing, yield_per_node
    if list_object.__class__ != list:
        raise Exception("%s must be a list" %(param))

    if func == "none":
        if len(list_object) != n_topo_nav_rows:
            raise Exception("%s must be a list of size %d for func='none'" %(param, n_topo_nav_rows))
        return des_param_list_check(param, list_object, n_topo_nav_rows, func="none")

    elif func == "copy":
        if len(list_object) == 1:
            return des_param_list_check(param, list_object, n_topo_nav_rows, func)
        elif len(list_object) == n_polytunnels:
            # same value for all rows of one polytunnel
            # get the list for each polytunnel and append them
            new_list = []

            for i in range(n_polytunnels):
                n_rows = n_farm_rows[i] + 1
                new_list += des_param_list_check(param, [list_object[i]], n_rows, func)
            return new_list

    elif func == "gauss":
        if len(list_object) != 2:
            raise Exception("%s must be a list of size 2 for func='gauss'" %(param))
        return des_param_list_check(param, list_object, n_topo_nav_rows, func)

    else:
        raise Exception("func must be none, copy or gauss")

def graph_param_to_poly_list(param, list_object, n_polytunnels, func):
    """convert a given list of values to a list of size n_polytunnels

    Keyword arguments:

    param -- string name of parameter
    list_object -- original list
    n_polytunnels -- number of polytunnels
    func -- function to be used for extending to n_polytunnels values
    """
    if list_object.__class__ != list:
        raise Exception("%s must either be a list" %(param))

    if n_polytunnels == 0 or n_polytunnels == 1:
        # func can be ignored as only one value is necessary
        if len(list_object) != 1:
            raise Exception("%s must be a list of size 1 for n_polytunnels = %d" %(param, n_polytunnels))
        return list_object

    else:
        if func == "none":
            if len(list_object) != n_polytunnels:
                raise Exception("%s must be a list of size %d for func 'none'" %(param, n_polytunnels))
            return list_object

        elif func == "copy":
            if len(list_object) != 1:
                raise Exception("%s must be a list of size 1 for func 'copy'" %(param))
            return [list_object[0] for i in range(n_polytunnels)]

def get_fork_map_config_parameters():
    """get_fork_map_config_parameters: get the configuration parameters requried for
    making a fork map from the rosparam server
    """
    # assume check_des_config_parameters.py is run and all parameters are present
    ns = rospy.get_namespace()

    n_polytunnels = rospy.get_param(ns + "rasberry_des_config/n_polytunnels")

    n_farm_rows_func = rospy.get_param(ns + "rasberry_des_config/n_farm_rows_func")
    _n_farm_rows = rospy.get_param(ns + "rasberry_des_config/n_farm_rows")
    n_farm_rows = graph_param_to_poly_list(ns + "rasberry_des_config/n_farm_rows", _n_farm_rows,
                                           n_polytunnels, n_farm_rows_func)

    # n_rows+1 picking rows are needed, all except first and last (in a polytunnel)
    # rows are forward and reverse. first and last are forward/reverse only
    if n_polytunnels == 0:
        n_topo_nav_rows = n_farm_rows[0] + 1
    else:
        n_topo_nav_rows = 0
        for i in range(n_polytunnels):
            n_topo_nav_rows += n_farm_rows[i] + 1

    head_row_node_dist_func = rospy.get_param(ns + "rasberry_des_config/head_row_node_dist_func")
    _head_row_node_dist = rospy.get_param(ns + "rasberry_des_config/head_row_node_dist")
    head_row_node_dist = graph_param_list_check(ns + "rasberry_des_config/head_row_node_dist",
                                                _head_row_node_dist, n_polytunnels,
                                                n_farm_rows, n_topo_nav_rows,
                                                head_row_node_dist_func)

    head_node_x_func = rospy.get_param(ns + "rasberry_des_config/head_node_x_func")
    _head_node_x = rospy.get_param(ns + "rasberry_des_config/head_node_x")
    head_node_x = graph_param_list_check(ns + "rasberry_des_config/head_node_x",
                                         _head_node_x, n_polytunnels,
                                         n_farm_rows, n_topo_nav_rows,
                                         head_node_x_func)

    row_node_dist_func = rospy.get_param(ns + "rasberry_des_config/row_node_dist_func")
    _row_node_dist = rospy.get_param(ns + "rasberry_des_config/row_node_dist")
    row_node_dist = graph_param_list_check(ns + "rasberry_des_config/row_node_dist",
                                           _row_node_dist, n_polytunnels,
                                           n_farm_rows, n_topo_nav_rows,
                                           row_node_dist_func)

    row_length_func = rospy.get_param(ns + "rasberry_des_config/row_length_func")
    _row_length = rospy.get_param(ns + "rasberry_des_config/row_length")
    row_length = graph_param_list_check(ns + "rasberry_des_config/row_length",
                                        _row_length, n_polytunnels,
                                        n_farm_rows, n_topo_nav_rows,
                                        row_length_func)

    # row_spacing is assumed to be the spacing between the
    # farm rows or between the edge and a row, or between two rows
    # navigation rows should be at the middle of this spacing.
    row_spacing_func = rospy.get_param(ns + "rasberry_des_config/row_spacing_func")
    _row_spacing = rospy.get_param(ns + "rasberry_des_config/row_spacing")
    row_spacing = graph_param_list_check(ns + "rasberry_des_config/row_spacing",
                                         _row_spacing, n_polytunnels,
                                         n_farm_rows, n_topo_nav_rows,
                                         row_spacing_func)

    if rospy.has_param(ns + "rasberry_des_config/dist_to_cold_storage"):
        dist_to_cold_storage = rospy.get_param(ns + "rasberry_des_config/dist_to_cold_storage")

    config_params = {}
    config_params["n_polytunnels"] = n_polytunnels
    config_params["n_farm_rows"] = n_farm_rows
    config_params["n_topo_nav_rows"] = n_topo_nav_rows
    config_params["head_row_node_dist"] = head_row_node_dist
    config_params["head_node_x"] = head_node_x
    config_params["row_node_dist"] = row_node_dist
    config_params["row_length"] = row_length
    config_params["row_spacing"] = row_spacing
    if rospy.has_param(ns + "rasberry_des_config/dist_to_cold_storage"):
        config_params["dist_to_cold_storage"] = dist_to_cold_storage

    return config_params

def get_des_config_parameters(map_from_db=False, n_pickers=None, n_robots=None):
    """get_des_config_parameters: Get the parameters for configuring the discrete event simulation
    from the rosparam server
    """
    ns = rospy.get_namespace()
    config_params = {}

    if not map_from_db:
        # n_rows, n_topo_nav_rows, row_node_dist,
        # row_length, row_spacing, head_row_node_dist
        config_params = get_fork_map_config_parameters()

    else:
        # TODO: Check whether reading some parameters from db is possible
        pass

    if not config_params:
        raise Exception("topo_map config parameters missing")

    n_polytunnels = config_params["n_polytunnels"]
    n_farm_rows = config_params["n_farm_rows"]
    n_topo_nav_rows = config_params["n_topo_nav_rows"]

    # remaining parameters

    des_env = rospy.get_param(ns + "rasberry_des_config/des_env")

    if n_pickers is None:
        n_pickers = rospy.get_param(ns + "rasberry_des_config/n_pickers")

    # picker parameters - des parameters
    picker_picking_rate_func = rospy.get_param(ns + "rasberry_des_config/picker_picking_rate_func")

    _picker_picking_rate = rospy.get_param(ns + "rasberry_des_config/picker_picking_rate")
    picker_picking_rate = des_param_list_check(ns + "rasberry_des_config/picker_picking_rate",
                                               _picker_picking_rate, n_pickers,
                                               picker_picking_rate_func)

    picker_transportation_rate_func = rospy.get_param(ns + "rasberry_des_config/picker_transportation_rate_func")
    _picker_transportation_rate = rospy.get_param(ns + "rasberry_des_config/picker_transportation_rate")
    picker_transportation_rate = des_param_list_check(ns + "rasberry_des_config/picker_transportation_rate",
                                                      _picker_transportation_rate, n_pickers,
                                                      picker_transportation_rate_func)

    picker_max_n_trays_func = rospy.get_param(ns + "rasberry_des_config/picker_max_n_trays_func")
    _picker_max_n_trays = rospy.get_param(ns + "rasberry_des_config/picker_max_n_trays")
    picker_max_n_trays = des_param_list_check(ns + "rasberry_des_config/picker_max_n_trays",
                                              _picker_max_n_trays, n_pickers,
                                              picker_max_n_trays_func)

    picker_unloading_time_func = rospy.get_param(ns + "rasberry_des_config/picker_unloading_time_func")
    _picker_unloading_time = rospy.get_param(ns + "rasberry_des_config/picker_unloading_time")
    picker_unloading_time = des_param_list_check(ns + "rasberry_des_config/picker_unloading_time",
                                                 _picker_unloading_time, n_pickers,
                                                 picker_unloading_time_func)

    tray_capacity = rospy.get_param(ns + "rasberry_des_config/tray_capacity")

    # yield - graph parameter
    yield_per_node_func = rospy.get_param(ns + "rasberry_des_config/yield_per_node_func")
    _yield_per_node = rospy.get_param(ns + "rasberry_des_config/yield_per_node")
    yield_per_node = graph_param_list_check(ns + "rasberry_des_config/yield_per_node",
                                            _yield_per_node, n_polytunnels, n_farm_rows,
                                            n_topo_nav_rows, yield_per_node_func)

    n_local_storages = rospy.get_param(ns + "rasberry_des_config/n_local_storages")

    if n_robots is None:
        n_robots = rospy.get_param(ns + "rasberry_des_config/n_robots")

    # robot parameters - des parameters
    _robot_transportation_rate = rospy.get_param(ns + "rasberry_des_config/robot_transportation_rate")
    robot_transportation_rate = des_param_list_check(ns + "rasberry_des_config/robot_transportation_rate",
                                                     _robot_transportation_rate, n_robots)

    robot_max_n_trays_func = rospy.get_param(ns + "rasberry_des_config/robot_max_n_trays_func")
    _robot_max_n_trays = rospy.get_param(ns + "rasberry_des_config/robot_max_n_trays")
    robot_max_n_trays = des_param_list_check(ns + "rasberry_des_config/robot_max_n_trays",
                                             _robot_max_n_trays, n_robots,
                                             robot_max_n_trays_func)

    _robot_unloading_time = rospy.get_param(ns + "rasberry_des_config/robot_unloading_time")
    robot_unloading_time = des_param_list_check(ns + "rasberry_des_config/robot_unloading_time",
                                                _robot_unloading_time, n_robots)

    config_params["des_env"] = des_env
    config_params["n_pickers"] = n_pickers
    config_params["picker_picking_rate"] = picker_picking_rate
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
