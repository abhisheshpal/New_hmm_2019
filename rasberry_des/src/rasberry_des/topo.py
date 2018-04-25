#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info:
# ----------------------------------


import numpy
import rospy
import topological_navigation.tmap_utils
import topological_navigation.route_search
import strands_navigation_msgs.msg
import rasberry_des.config_utils


class TopologicalForkGraph(object):
    """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.
    """
    def __init__(self, n_farm_rows, n_topo_nav_rows, _node_yields, verbose):
        """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.

        Keyword arguments:

        n_topo_nav_row -- number of toplogical navigation rows, int
        row_ids -- identifications of the navigation rows, list of size n_topo_nav_rows
        _node_yields -- yields per node distance for each row, list of size n_topo_nav_rows or 1
        """
        ns = rospy.get_namespace()
        self.row_ids = ["row-%02d" %(i) for i in range(n_topo_nav_rows)]
        self.n_farm_rows = n_farm_rows
        self.n_topo_nav_rows = n_topo_nav_rows
        # half_rows: rows requiring picking in one direction
        self.half_rows = set((self.row_ids[0], self.row_ids[-1]))
        self.head_nodes = {}        # {row_id:head_node}
        self.row_nodes = {}         # {row_id:[row_nodes]}
        # row_info {row_id:[head_node, start_node, end_node, local_storage_node]}
        self.row_info = {}
        # yield_at_node {node_id:yield_at_node}
        self.yield_at_node = {}
        # local storage nodes
        self.local_storages = {}
        self.local_storage_nodes = {row_id:None for row_id in self.row_ids}
        self.cold_storage = None
        self.cold_storage_node = None

        self.use_local_storage = True # if False, store at cold storage

        self.verbose = verbose

        for i in range(10):
            try:
                self.topo_map = rospy.wait_for_message(ns + "topological_map", strands_navigation_msgs.msg.TopologicalMap, timeout=10)
            except:
                rospy.logerr(ns + "topological_map topic is not published?")
                rospy.sleep(0.1)
            else:
                self.loginfo("TopologicalForkGraph object ready")
                break

        if self.topo_map:
            self.get_row_info()
            # local storages should be set by calling set_local_storages externally
            self.set_node_yields(_node_yields)
            self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)
        else:
            rospy.ROSException(ns + "topological_map topic not received")

    def set_node_yields(self, _node_yields):
        """set_node_yields: Set the yields at each node from the node yields
        given for each row / all rows

        Keyword arguments:

        _node_yields -- yields per node distance for each row,
                            list of size n_topo_nav_rows or 1
        """
        node_yield_in_row = rasberry_des.config_utils.param_list_to_dict("node_yields", _node_yields, self.row_ids)

        for row_id in self.row_ids:
            n_row_nodes = len(self.row_nodes[row_id])

            for j in range(n_row_nodes):
                node_id = self.row_nodes[row_id][j]
                # yield from each row node to next row node
                if j != n_row_nodes - 1:
#                    self.yield_at_node[node_id] = numpy.random.logistic(node_yield_in_row[row_id])
                    self.yield_at_node[node_id] = node_yield_in_row[row_id]
                else:
                    # between the last two nodes, the distance could be smaller than node_dist
                    row_node_dist = self.get_distance_between_nodes(self.row_nodes[row_id][0],
                                                                    self.row_nodes[row_id][1])
                    last_node_dist = self.get_distance_between_nodes(self.row_nodes[row_id][-2],
                                                                     self.row_nodes[row_id][-1])
#                    self.yield_at_node[node_id] = numpy.random.logistic((node_yield_in_row[row_id] * last_node_dist) / row_node_dist)
                    self.yield_at_node[node_id] = (node_yield_in_row[row_id] * last_node_dist) / row_node_dist

    def set_local_storages(self, local_storages):
        """set local_storage_nodes and local_storages

        Keyword arguments:

        local_storages -- simpy.Resource object list
        """
        # reset
        self.local_storage_nodes = {row_id:None for row_id in self.row_ids}
        self.local_storages = {}

        n_local_storages = len(local_storages)
        # set local storage nodes
        storage_row_groups = numpy.array_split(numpy.arange(self.n_topo_nav_rows), n_local_storages)

        for i in range(n_local_storages):
            start_row = storage_row_groups[i][0]
            end_row = storage_row_groups[i][-1]
            storage_row = "hn-%02d" %(start_row + int((end_row - start_row) / 2))

            for row in storage_row_groups[i]:
                self.local_storage_nodes["row-%02d" %(row)] = storage_row
            self.local_storages[storage_row] = local_storages[i]

        for row_id in self.row_ids:
            self.row_info[row_id][3] = self.local_storage_nodes[row_id]

    def set_cold_storage(self, cold_storage):
        """set cold_storage_node and cold_storage

        Keyword arguments:

        cold_storage -- simpy.Resource object
        """
        self.cold_storage_node = "cold_storage"
        self.cold_storage = cold_storage
        self.use_local_storage = False

    def get_row_info(self, ):
        """get_row_info: Get information about each row
        {row_id: [head_node, start_node, end_node, local_storage_node]}

        Also sets
          head_nodes {row_id:head_node}
          row_nodes {row_id:[row_nodes]}
          local_storages = {local_storage_node:simpy.Resource}

        Keyword arguments:

        local_storages -- simpy.Resource objects, list
        """
        # TODO: meta information is not queried from the db now.
        # The row and head node names are hard coded now
        # An ugly way to sort the nodes is implemented
        # get_nodes in topological_utils.queries might be useful to get nodes with same tag
        self.head_nodes = {"row-%02d" %(i):"hn-%02d" %(i) for i in range(self.n_topo_nav_rows)}
        self.row_nodes = {"row-%02d" %(i):[] for i in range(self.n_topo_nav_rows)}

        for node in self.topo_map.nodes:
            for i in range(self.n_topo_nav_rows):
                if "rn-%02d" %(i) in node.name:
                    self.row_nodes["row-%02d" %(i)].append(node.name)

        for row_id in self.row_ids:
            self.row_nodes[row_id].sort()
            # local_storage_nodes should be modified by calling set_local_storages
            self.row_info[row_id] = [self.head_nodes[row_id],
                                     self.row_nodes[row_id][0],
                                     self.row_nodes[row_id][-1],
                                     self.local_storage_nodes[row_id]]

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:
        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_search.search_route(start_node, goal_node)
        route_nodes = route.source
        route_edges = route.edge_id

        edge_to_goal = self.get_edges_between_nodes(route_nodes[-1], goal_node)
        route_edges.append(edge_to_goal[0])
        route_nodes.append(goal_node)

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.get_distance_between_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return topological_navigation.tmap_utils.get_node(self.topo_map, node)

    def get_distance_between_nodes(self, from_node, to_node):
        """get_distance_between_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node(from_node_obj, to_node_obj)

    def get_edges_between_nodes(self, from_node, to_node):
        """get_edges_between_nodes: Given names of two nodes, return the direct edges
        between their node objects. A wrapper for the get_edges_between function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        edge_ids = []
        edges = topological_navigation.tmap_utils.get_edges_between(self.topo_map, from_node, to_node)
        for edge in edges:
            edge_ids.append(edge.edge_id)
        return edge_ids

    def loginfo(self, msg):
        """log info based on a flag"""
        if self.verbose:
            rospy.loginfo(msg)
