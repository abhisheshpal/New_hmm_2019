#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info:
# ----------------------------------


import topological_navigation.tmap_utils
import topological_navigation.route_search
import strands_navigation_msgs.msg
import rospy
import numpy

class TopologicalForkGraph(object):
    """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.
    """
    def __init__(self, n_topo_nav_rows, row_ids, _node_yields, local_storages):
        """TopologicalForkGraph: A class to store and retreive information of topological map,
        stored in the mongodb, necessary for the discrete event simulations.Assumes a fork map with
        one head lane and different rows.

        Keyword arguments:

        n_topo_nav_row -- number of toplogical navigation rows, int
        row_ids -- identifications of the navigation rows, list of size n_topo_nav_rows
        _node_yields -- yields per node distance for each row, list of size n_topo_nav_rows or 1
        local_storages -- simpy.Resource objects, list
        """
        ns = rospy.get_namespace()
        self.row_ids = row_ids
        self.n_topo_nav_rows = n_topo_nav_rows
        self.head_nodes = {}        # {row_id:head_node_name}
        self.row_nodes = {}         # {row_id:[row_node_names]}
        # row_info {row_id:[head_node, start_node, end_node, row_node_dist, last_node_dist]}
        self.row_info = {}
        # yield_at_node {node_id:yield_at_node}
        self.yield_at_node = {}
        # local storage nodes
        self.local_storages = {}
        self.local_storage_nodes = {row_id:None for row_id in self.row_ids}

        for i in range(10):
            try:
                self.topo_map = rospy.wait_for_message(ns + "topological_map", strands_navigation_msgs.msg.TopologicalMap, timeout=10)
            except:
                rospy.logerr(ns + "topological_map topic is not published?")
                rospy.sleep(0.1)
            else:
                rospy.loginfo("TopologicalForkGraph object ready")
                break

        if self.topo_map:
            self.get_row_info()
            self.set_node_yields(_node_yields)
            self.set_local_storages(local_storages)
            self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)
        else:
            rospy.ROSException(ns + "topological_map topic not received")

    def set_node_yields(self, _node_yields):
        """set_node_yields: Set the yields at each node from the node yields given for each row / all rows

        Keyword arguments:

        _node_yields -- yields per node distance for each row, list of size n_topo_nav_rows or 1
        """
        if _node_yields.__class__ == list:
            if len(_node_yields) == self.n_topo_nav_rows:
                node_yield_in_row = {self.row_ids[i]:_node_yields[i] for i in range(self.n_topo_nav_rows)}
            elif len(_node_yields) == 1:
                node_yield_in_row = {self.row_ids[i]:_node_yields[0] for i in range(self.n_topo_nav_rows)}
        else:
            raise TypeError("_node_yields must be list of length %d or 1" %(self.n_topo_nav_rows))

        for row_id in self.row_ids:
            n_row_nodes = len(self.row_nodes[row_id])

            for j in range(n_row_nodes):
                node_id = self.row_nodes[row_id][j]
                # yield from each row node to next row node
                if j != n_row_nodes - 1:
                    self.yield_at_node[node_id] = numpy.random.logistic(node_yield_in_row[row_id])
                else:
                    # between the last two nodes, the distance could be smaller than node_dist
                    row_node_dist = self.row_info[row_id][3]
                    last_node_dist = self.row_info[row_id][4]
                    self.yield_at_node[node_id] = numpy.random.logistic((node_yield_in_row[row_id] * last_node_dist) / row_node_dist)

    def set_local_storages(self, local_storages):
        """set_local_storages: set the local_storage_nodes {row_id:storage_node_id} and
        local_storages {storage_node_id: simpy.Resource}

        Keyword arguments:

        local_storages -- simpy.Resource objects, list
        """
        n_local_storages = len(local_storages)

        storage_row_groups = numpy.array_split(numpy.arange(self.n_topo_nav_rows), n_local_storages)

        for i in range(n_local_storages):
            start_row = storage_row_groups[i][0]
            end_row = storage_row_groups[i][-1]
            storage_row = "hn-%02d" %(start_row + int((end_row - start_row) / 2))

            for row in storage_row_groups[i]:
                self.local_storage_nodes["row-%02d" %(row)] = storage_row
            self.local_storages[storage_row] = local_storages[i]

    def get_row_info(self, ):
        """get_row_info: Get information about each row
        {row_id: [head_node_name, start_node, end_node, node_dist, last_node_dist]}
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
            n_row_nodes = len(self.row_nodes[row_id])
            if n_row_nodes > 2:
                from_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                       self.row_nodes[row_id][0])
                to_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                     self.row_nodes[row_id][1])
                row_node_dist = topological_navigation.tmap_utils.get_distance_to_node(from_node,
                                                                                       to_node)

                from_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                       self.row_nodes[row_id][-2])
                to_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                     self.row_nodes[row_id][-1])
                last_node_dist = topological_navigation.tmap_utils.get_distance_to_node(from_node,
                                                                                       to_node)
            elif n_row_nodes == 2:
                from_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                       self.row_nodes[row_id][0])
                to_node = topological_navigation.tmap_utils.get_node(self.topo_map,
                                                                     self.row_nodes[row_id][1])
                row_node_dist = topological_navigation.tmap_utils.get_distance_to_node(from_node,
                                                                                       to_node)

                last_node_dist = row_node_dist
            else:
                row_node_dist = last_node_dist = 0.

            self.row_info[row_id] = [self.head_nodes[row_id],
                                     self.row_nodes[row_id][0],
                                     self.row_nodes[row_id][-1],
                                     row_node_dist,
                                     last_node_dist]

    def get_path_details(self, start_node_name, goal_node_name):
        """get route_nodes, route_edges and route_distance from start_node_name to goal_node_name

        Keyword arguments:
        start_node_name -- name of the starting node
        goal_node_name -- name of the goal node
        """
        route_distance = 0.0
        route = self.route_search.search_route(start_node_name, goal_node_name)
        route_nodes = route.source
        route_edges = route.edge_id

        edge_to_goal = topological_navigation.tmap_utils.get_edges_between(self.topo_map, route_nodes[-1], goal_node_name)
        route_edges.append(edge_to_goal[0])
        route_nodes.append(goal_node_name)

        for i in range(len(route_nodes) - 1):
            from_node = topological_navigation.tmap_utils.get_node(self.topo_map, route_nodes[i])
            to_node = topological_navigation.tmap_utils.get_node(self.topo_map, route_nodes[i+1])
            route_distance += topological_navigation.tmap_utils.get_distance_to_node(from_node, to_node)

        return (route_nodes, route_edges, route_distance)

    def get_node_xy(self, node_name):
        """get_node_pose: Given a node name return its x and y coordinates"""
        node = topological_navigation.tmap_utils.get_node(self.topo_map, node_name)
        return (node.pose.position.x, node.pose.position.y)
