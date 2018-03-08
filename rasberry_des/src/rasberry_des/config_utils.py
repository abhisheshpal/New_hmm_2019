#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

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
    if not rospy.has_param(ns + "rasberry_des_config/head_node_y"):
        missing_params.append("rasberry_des_config/head_node_y")
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
    if not rospy.has_param(ns + "rasberry_des_config/transportation_rate"):
        missing_params.append("rasberry_des_config/transportation_rate")
    if not rospy.has_param(ns + "rasberry_des_config/max_n_trays"):
        missing_params.append("rasberry_des_config/max_n_trays")
    if not rospy.has_param(ns + "rasberry_des_config/loading_time"):
        missing_params.append("rasberry_des_config/loading_time")
    if not rospy.has_param(ns + "rasberry_des_config/tray_capacity"):
        missing_params.append("rasberry_des_config/tray_capacity")
    if not rospy.has_param(ns + "rasberry_des_config/yield_per_node"):
        missing_params.append("rasberry_des_config/yield_per_node")
    return missing_params

def get_fork_map_config_parameters():
    # assume check_des_config_parameters.py is run and all parameters are present
    ns = rospy.get_namespace()
    n_farm_rows = rospy.get_param(ns + "rasberry_des_config/n_farm_rows")

    # Half or full rows
    # If half rows, n_rows-1 picking rows are needed, all forward and reverse
    # If full rows, n_rows+1 picking rows are needed, all except first and last
    # rows are forward and reverse. first and last are forward/reverse only
    half_rows = rospy.get_param(ns + "rasberry_des_config/half_rows")

    if (half_rows):
        n_topo_nav_rows = n_farm_rows - 1
    else:
        n_topo_nav_rows = n_farm_rows + 1

    _head_row_node_dist = rospy.get_param(ns + "rasberry_des_config/head_row_node_dist")
    if (_head_row_node_dist.__class__ == list):
        # if there are n_rows values, take
        if (len(_head_row_node_dist) == n_topo_nav_rows) or (len(_head_row_node_dist) == 1):
            head_row_node_dist = _head_row_node_dist
        else:
            rospy.ROSException("%srasberry_des_config/head_row_node_dist must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/head_row_node_dist must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    _head_node_y = rospy.get_param(ns + "rasberry_des_config/head_node_y")
    if (_head_node_y.__class__ == list):
        # if there are n_rows values, take
        if (len(_head_node_y) == n_topo_nav_rows) or (len(_head_node_y) == 1):
            head_node_y = _head_node_y
        else:
            rospy.ROSException("%srasberry_des_config/head_node_y must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/head_node_y must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    _row_node_dist = rospy.get_param(ns + "rasberry_des_config/row_node_dist")
    if (_row_node_dist.__class__ == list):
        # if there are n_rows values, take
        if (len(_row_node_dist) == n_topo_nav_rows) or (len(_row_node_dist) == 1):
            row_node_dist = _row_node_dist
        else:
            rospy.ROSException("%srasberry_des_config/row_node_dist must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/row_node_dist must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    # TODO: Now row_length is assumed to be for navigation rows
    # This in turn will assume picking is happening on berry rows on both sides
    # This should ideally be the length of berry rows, with picking from either
    # or both sides depending on the length of the berry rows.
    _row_length = rospy.get_param(ns + "rasberry_des_config/row_length")
    if (_row_length.__class__ == list):
        # if there are n_rows values, take
        if (len(_row_length) == n_topo_nav_rows) or (len(_row_length) == 1):
            row_length = _row_length
        else:
            rospy.ROSException("%srasberry_des_config/row_length must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/row_length must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    # TODO: row_spacing is assumed to be the spacing between the
    # farm rows or between the edge and a row, depending on the berry rows
    # are half or full. the navigation row nodes should be at the middle of
    # this spacing. make sure it makes sense later.
    _row_spacing = rospy.get_param(ns + "rasberry_des_config/row_spacing")
    if (_row_length.__class__ == list):
        # if there are n_rows values, take
        if (len(_row_spacing) == n_topo_nav_rows) or (len(_row_spacing) == 1):
            row_spacing = _row_spacing
        else:
            rospy.ROSException("%srasberry_des_config/row_spacing must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/row_spacing must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    return (n_farm_rows, half_rows, n_topo_nav_rows, head_row_node_dist, head_node_y,
            row_node_dist, row_length, row_spacing)

def get_des_config_parameters(map_from_db=False):
    ns = rospy.get_namespace()
    config_params = []

    if not map_from_db:
        # n_rows, half_rows, n_topo_nav_rows, row_node_dist,
        # row_length, row_spacing, head_row_node_dist
        config_params = list(get_fork_map_config_parameters())

    else:
        # TODO: Check whether reading some parameters from db is possible
        pass

    if len(config_params) == 0:
        rospy.ROSException("topo_map config parameters missing")

    n_topo_nav_rows = config_params[2]

    # remaining parameters

    des_env = rospy.get_param(ns + "rasberry_des_config/des_env")

    n_pickers = rospy.get_param(ns + "rasberry_des_config/n_pickers")

    _picking_rate = rospy.get_param(ns + "rasberry_des_config/picking_rate")
    if (_picking_rate.__class__ == list):
        if (len(_picking_rate) == n_pickers) or (len(_picking_rate) == 1):
            picking_rate = _picking_rate
        else:
            rospy.ROSException("%srasberry_des_config/picking_rate must either be an array of size %d or 1" %(ns, n_pickers))
    else:
        rospy.ROSException("%srasberry_des_config/picking_rate must either be an array of size %d or 1" %(ns, n_pickers))

    _transportation_rate = rospy.get_param(ns + "rasberry_des_config/transportation_rate")
    if (_transportation_rate.__class__ == list):
        if (len(_transportation_rate) == n_pickers) or (len(_transportation_rate) == 1):
            transportation_rate = _transportation_rate
        else:
            rospy.ROSException("%srasberry_des_config/transportation_rate must either be an array of size %d or 1" %(ns, n_pickers))
    else:
        rospy.ROSException("%srasberry_des_config/transportation_rate must either be an array of size %d or 1" %(ns, n_pickers))

    _max_n_trays = rospy.get_param(ns + "rasberry_des_config/max_n_trays")
    if (_max_n_trays.__class__ == list):
        if (len(_max_n_trays) == n_pickers) or (len(_max_n_trays) == 1):
            max_n_trays = _max_n_trays
        else:
            rospy.ROSException("%srasberry_des_config/max_n_trays must either be an array of size %d or 1" %(ns, n_pickers))
    else:
        rospy.ROSException("%srasberry_des_config/max_n_trays must either be an array of size %d or 1" %(ns, n_pickers))

    _loading_time = rospy.get_param(ns + "rasberry_des_config/loading_time")
    if (_loading_time.__class__ == list):
        if (len(_loading_time) == n_pickers) or (len(_loading_time) == 1):
            loading_time = _loading_time
        else:
            rospy.ROSException("%srasberry_des_config/loading_time must either be an array of size %d or 1" %(ns, n_pickers))
    else:
        rospy.ROSException("%srasberry_des_config/loading_time must either be an array of size %d or 1" %(ns, n_pickers))

    tray_capacity = rospy.get_param(ns + "rasberry_des_config/tray_capacity")

    _yield_per_node = rospy.get_param(ns + "rasberry_des_config/yield_per_node")
    if (_yield_per_node.__class__ == list):
        if (len(_yield_per_node) == n_topo_nav_rows) or (len(_yield_per_node) == 1):
            yield_per_node = _yield_per_node
        else:
            rospy.ROSException("%srasberry_des_config/yield_per_node must either be an array of size %d or 1" %(ns, n_topo_nav_rows))
    else:
        rospy.ROSException("%srasberry_des_config/yield_per_node must either be an array of size %d or 1" %(ns, n_topo_nav_rows))

    config_params.append(des_env)
    config_params.append(n_pickers)
    config_params.append(picking_rate)
    config_params.append(transportation_rate)
    config_params.append(max_n_trays)
    config_params.append(loading_time)
    config_params.append(tray_capacity)
    config_params.append(yield_per_node)

    return config_params