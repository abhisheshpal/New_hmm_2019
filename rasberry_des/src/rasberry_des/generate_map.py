#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import strands_navigation_msgs.srv
import geometry_msgs.msg
import numpy


def generate_fork_map(n_farm_rows, half_rows, n_topo_nav_rows, _head_row_node_dist,
                      _head_node_y, _row_node_dist, _row_length, _row_spacing):
    """generate fork map by creating nodes and edges. Use fork_map_generator
    node to call this function.
    """
    # ros service proxies
    # add nodes
    rospy.loginfo("waiting for service /topological_map_manager/add_topological_node")
    rospy.wait_for_service("/topological_map_manager/add_topological_node")
    add_node = rospy.ServiceProxy("/topological_map_manager/add_topological_node", strands_navigation_msgs.srv.AddNode)
    # add node tags
    rospy.loginfo("waiting for service /topological_map_manager/add_tag_to_node")
    rospy.wait_for_service("/topological_map_manager/add_tag_to_node")
    add_node_tag = rospy.ServiceProxy("/topological_map_manager/add_tag_to_node", strands_navigation_msgs.srv.AddTag)
    # add edges
    rospy.loginfo("waiting for service /topological_map_manager/add_edges_between_nodes")
    rospy.wait_for_service("/topological_map_manager/add_edges_between_nodes")
    add_edges = rospy.ServiceProxy("/topological_map_manager/add_edges_between_nodes", strands_navigation_msgs.srv.AddEdge)

    pose = geometry_msgs.msg.Pose()
    pose.orientation.w = 1

    row_ids = ["row_%02d" %(i) for i in range(n_topo_nav_rows)]

    if _head_row_node_dist.__class__ == list:
        if len(_head_row_node_dist) == n_topo_nav_rows:
            head_row_node_dist = {row_ids[i]:_head_row_node_dist[i] for i in range(n_topo_nav_rows)}
        elif len(_head_row_node_dist) == 1:
            head_row_node_dist = {row_ids[i]:_head_row_node_dist[0] for i in range(n_topo_nav_rows)}
    else:
        raise TypeError("head_row_node_dist must be list of length %d or 1" %(n_topo_nav_rows))

    if _head_node_y.__class__ == list:
        if len(_head_node_y) == n_topo_nav_rows:
            head_node_y = {row_ids[i]:_head_node_y[i] for i in range(n_topo_nav_rows)}
        elif len(_head_node_y) == 1:
            head_node_y = {row_ids[i]:_head_node_y[0] for i in range(n_topo_nav_rows)}
    else:
        raise TypeError("head_node_y must be list of length %d or 1" %(n_topo_nav_rows))

    if _row_node_dist.__class__ == list:
        if len(_row_node_dist) == n_topo_nav_rows:
            row_node_dist = {row_ids[i]:_row_node_dist[i] for i in range(n_topo_nav_rows)}
        elif len(_row_node_dist) == 1:
            row_node_dist = {row_ids[i]:_row_node_dist[0] for i in range(n_topo_nav_rows)}
    else:
        raise TypeError("row_node_dist must be list of length %d or 1" %(n_topo_nav_rows))

    if _row_length.__class__ == list:
        if len(_row_length) == n_topo_nav_rows:
            row_length = {row_ids[i]:_row_length[i] for i in range(n_topo_nav_rows)}
        elif len(_row_length) == 1:
            row_length = {row_ids[i]:_row_length[0] for i in range(n_topo_nav_rows)}
    else:
        raise TypeError("row_length must be list of length %d or 1" %(n_topo_nav_rows))

    if _row_spacing.__class__ == list:
        if len(_row_spacing) == n_topo_nav_rows:
            row_spacing = {row_ids[i]:_row_spacing[i] for i in range(n_topo_nav_rows)}
        elif len(_row_spacing) == 1:
            row_spacing = {row_ids[i]:_row_spacing[0] for i in range(n_topo_nav_rows)}
    else:
        raise TypeError("row_spacing must be list of length %d or 1" %(n_topo_nav_rows))

    # 1. create the nodes - head_nodes and row_nodes
    x = []
    for i in range(n_topo_nav_rows):
        row_id = row_ids[i]
        # words in node names are separated with -
        # words in tags are separated with _
        # words in edge_ids are separated with _
        head_node = "hn-%02d" %(i)

        if i == 0:
            x.append(row_spacing[row_id] / 2.)
        else:
            prev_row_id = row_ids[i - 1]
            x.append(x[-1] + row_spacing[prev_row_id] / 2. + row_spacing[row_id] / 2.)
        y = head_node_y[row_id]

        pose.position.x = x[-1]
        pose.position.y = y

        add_node(head_node, pose)
        add_node_tag("head_%s" %(row_id), [head_node])

        # add edges between head nodes
        if i > 0:
            prev_head_node = "hn-%02d" %(i-1)
            add_edges(prev_head_node, head_node, "move_base", "edge_heads_%02d_%02d" %(i - 1, i))
            add_edges(head_node, prev_head_node, "move_base", "edge_heads_%02d_%02d" %(i, i - 1))

        # row length can be different for different rows
        # 1 is for the end node, which is not produced in numpy.arange
        if row_length[row_id] > 0.:
            n_row_nodes = len(numpy.arange(0, row_length[row_id], row_node_dist[row_id])) + 1
        else:
            n_row_nodes = 0

        for j in range(n_row_nodes):
            row_node = "rn-%02d-%02d" %(i, j)

            if j == 0:
                y += head_row_node_dist[row_id]
            else:
                # row_length may not be an exact multiple of row_node_dist between the last two nodes
                y += row_node_dist[row_id] if j != n_row_nodes - 1 else (row_length[row_id] - (j - 1) * row_node_dist[row_id])
            pose.position.y = y

            add_node(row_node, pose)
            add_node_tag(row_id, [row_node])

        # 2. add edges
        for j in range(n_row_nodes - 1):
            curr_node = "rn-%02d-%02d" %(i, j)
            next_node = "rn-%02d-%02d" %(i, j+1)

            if j == 0:
                add_edges(head_node, curr_node, "move_base", "edge_head_row_%02d" %(i))
                add_edges(curr_node, head_node, "move_base", "edge_row_head_%02d" %(i))

            add_edges(curr_node, next_node, "move_base", "edge_%02d_%02d_%02d" %(i, j, j+1))
            add_edges(next_node, curr_node, "move_base", "edge_%02d_%02d_%02d" %(i, j+1, j))

