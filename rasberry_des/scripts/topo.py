#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info:
#        Graph - A simple topological graph representation with
#                   nodes,
#                   neighbours,
#                   edge_distances and
#                   A* method to find a path from one node to another.
#       Bi_Graph - A child class of Graph for bi-directional graph
#                   Only addition is adding both edges at the same time when a neighbour is added
# ----------------------------------


import math


class Node(object):
    """Node class definition"""
    def __init__(self, name, x, y, yield_at_node=0.):
        """Create a Node object

        Keyword arguments:
        name -- name of the object, a unique id
        x -- x coordinate
        y --y coordinate
        yield_at_node -- yield from this node to next node
        """
        self.name = name
        self.x = x
        self.y = y
        self.yield_at_node = yield_at_node

    def __str__(self, ):
        return self.name


class Graph(object):
    """Graph class definition"""
    def __init__(self, ):
        """Create an instance of Graph"""
        self.nodes = {}             # dict of Node objects
        self.neighbours = {}        # {NODE:[adjacent nodes]}
        self.edge_distances = {}      # {(NODE1, NODE2): distance/cost}

    def add_node(self, node, neighbours=None, edge_distances=None):
        """Add a node to the graph

        Keyword arguments:
        node -- NODE object
        neighbours -- list of adjacent Node objects, default: None
        edge_distances -- dict {adjacentNode : distance}, default: None
        """
        if node.name not in self.nodes:
            # node is not added earlier
            self.nodes[node.name] = node
            self.neighbours[node] = []
            if neighbours is not None:
                # edges
                for nb_node in neighbours:
                    # the edgeNode is added to the nodes in the graph
                    # without any neighbouring nodes.
                    # this is for forward compatibility with directed graphs.
                    # if an edge is bi-directional, the reverse should also be
                    # added explicitly.
                    if nb_node not in self.nodes:
                        self.add_node(nb_node)

                    self.neighbours[node].append(nb_node)
                    # either no distances are given or all are given
                    if edge_distances is None:
                        self.edge_distances[(node, nb_node)] = math.sqrt((node.x - nb_node.x)**2 +
                                                                         (node.y - nb_node.y)**2)
                    else:
                        self.edge_distances[(node, nb_node)] = edge_distances[nb_node]

        else:
            # if the node is already added to the graph.
            # this node might have been added when an edge from another
            # node was added earlier.
            if neighbours is not None:
                for nb_node in neighbours:
                    # if the neighbouring node is not present in nodes, add it
                    if nb_node not in self.nodes:
                        self.add_node(nb_node)
                    if nb_node not in self.neighbours[node]:
                        self.neighbours[node].append(nb_node)
                        # if the edge distance is given take that else Euclidian
                        # either no distances are given or all are given
                        if edge_distances is None:
                            self.edge_distances[(node, nb_node)] = math.sqrt((node.x - nb_node.x)**2 +
                                                                             (node.y - nb_node.y)**2)
                        else:
                            self.edge_distances[(node, nb_node)] = edge_distances[nb_node]

    def print_nodes(self, ):
        """print all node names"""
        print(":Nodes:\n------------")
        for node in self.nodes:
            print node
        print("------------")

    def print_neighbours(self, ):
        """print all nodes and their neighbours"""
        print(":Neighbours:\n------------")
        for node in self.nodes:
            nb_str = "%s: ["%(node)
            for nb_node in self.neighbours[self.nodes[node]]:
                nb_str += "%s, " %(nb_node)
            nb_str += "]"
            print(nb_str)
        print("------------")

    def a_star(self, start_node, goal_node):
        """run A* to get path from start_node to goal_node

        Keyword arguments:
        start_node -- starting Node object
        goal_node -- goal Node object
        """
        def get_min_f_score_node():
            """get the node with minimum f_score"""
            min_score = float("inf")
            min_score_node = None
            for node in open_set:
                if f_score[node] < min_score:
                    min_score = f_score[node]
                    min_score_node = node
            return min_score_node

        def reconstruct_path(node):
            """reconstruct path from node by tracing came_from"""
            par_node = came_from[node]
#            print(par_node)
            if par_node != None:
                edge_path.append(par_node)
                return reconstruct_path(par_node)
            else:
                return edge_path

        def heuristic_score(start_node, goal_node):
            """give a heuristic score based on Euclidean distance between two nodes"""
            # Euclidian disatance heuristic
            return math.sqrt((start_node.x - goal_node.x)**2 + (start_node.y - goal_node.y)**2)

        closed_set = set()               # nodes already evaluated
        open_set = set([start_node])        # set of currently discovered and not evaluated nodes
        came_from = {start_node: None}    # most efficient previous step
        g_score = {start_node: 0}         # cost of getting to the node from the goal node
        # total cost of getting to the goal_node from the start_node, through the node
        f_score = {start_node: heuristic_score(start_node, goal_node)}

        path_start_goal = None

        while len(open_set) != 0:
            curr_node = get_min_f_score_node()
            if curr_node is goal_node:
                edge_path = [goal_node]
                path_start_goal = reconstruct_path(curr_node)
                path_start_goal.reverse()
                return path_start_goal

            open_set.remove(curr_node)
            closed_set.add(curr_node)

            for nb_node in self.neighbours[curr_node]:
                if nb_node in closed_set:
                    # ignore neighbour, if already evaluated
                    continue

                if nb_node not in open_set:
                    # if neighbour was not evaluated, add to open_set
                    open_set.add(nb_node)

                tentative_g_score = g_score[curr_node] + self.edge_distances[(curr_node, nb_node)]
                if nb_node not in g_score:
                    # if g_score is not calculated for nb_node, go ahead
                    pass
                elif tentative_g_score >= g_score[nb_node]:
                    # this may not be better path as score increased
                    # (score proportional to distance, we need min)
                    # ignore this neighbour
                    continue

                came_from[nb_node] = curr_node
                g_score[nb_node] = tentative_g_score
                f_score[nb_node] = g_score[nb_node] + heuristic_score(nb_node, goal_node)

        return None

    def get_path_distance(self, edge_path_names):
        """given a path (list of names of connected nodes),
        get distance from start_node to goal_node

        Keyword arguments:
        edge_path_names -- list of names of nodes forming a sequence of connected edges"""
        path_distance = 0
        for i in range(len(edge_path_names) - 1):
            path_distance += self.edge_distances[(self.nodes[edge_path_names[i]],
                                                  self.nodes[edge_path_names[i+1]])]
        return path_distance

    def get_distance_to_node(self, x, y, node_name):
        """given an arbitrary point and a node, give the distance between them

        Keyword arguments:
        x -- x coordinate of arbitrary point
        y -- y coordinate of arbitrary point
        node_name -- name of node from which the distance to the arbitrary point is calculated"""
        return math.sqrt((x - self.nodes[node_name].x)**2 + (y - self.nodes[node_name].y)**2)

    def get_path(self, start_node_name, goal_node_name):
        """a wrapper for a_star method to use with node names.
        returns names of nodes in the path

        Keyword arguments:
        start_node_name -- name of the starting node
        goal_node_name -- name of the goal node
        """
        path_nodes = self.a_star(self.nodes[start_node_name], self.nodes[goal_node_name])
        path_node_names = []
        for node in path_nodes:
            path_node_names.append(node.name)
        return path_node_names

    def get_path_details(self, start_node_name, goal_node_name):
        """get path and path_dist from start_node_name to goal_node_name

        Keyword arguments:
        start_node_name -- name of the starting node
        goal_node_name -- name of the goal node
        """
        path_node_names = self.get_path(start_node_name, goal_node_name)
        path_dist = self.get_path_distance(path_node_names)
        return (path_node_names, path_dist)


class BiGraph(Graph):
    """ A subclass of Graph.
        Each edge can be bi-directional.
    """
    def __init__(self, ):
        Graph.__init__(self, )

    def add_node(self, node, neighbours=None, edge_distances=None):
        """extend the add_node of Graph by adding bi-directional edges at the same time

        Keyword arguments:
        node -- NODE object
        neighbours -- list of adjacent Node objects, default: None
        edge_distances -- dict {adjacentNode : distance}, default: None
        """
        # starts with the parent class's add_node method
        Graph.add_node(self, node, neighbours, edge_distances)
        # add bi-directional edges
        for nb_node in self.neighbours[node]:
            if node not in self.neighbours[nb_node]:
#                print(">>> node=%s, nb_node=%s" %(node, nb_node))
                self.neighbours[nb_node].append(node)
                self.edge_distances[(nb_node, node)] = self.edge_distances[(node, nb_node)]

if __name__ == "__main__":
    # A sample code for testing node distance calculation
    graph = BiGraph()
    a = Node("a", 0, 0, 0)
    b = Node("b", 1, 1, 0)
    c = Node("c", 2, 2, 0)
    d = Node("d", 3, 1, 0)
    e = Node("e", 5, 5, 0)
    f = Node("f", 0, 3, 0)
    g = Node("g", 1, 4, 0)
    h = Node("h", 3, 4, 0)
    graph.add_node(a, [b, f])
    graph.add_node(b, [a, c, f])
    graph.add_node(c, [b, d, g])
    graph.add_node(d, [c, h, e])
    graph.add_node(e, [d, h])
    graph.add_node(f, [a, b, g])
    graph.add_node(g, [c, f, h])
    graph.add_node(h, [d, e, g])

    edge_path = graph.a_star(a, e)
    for node in edge_path:
        print("%s" %(node))

    print(graph.print_nodes())
    print(graph.print_neighbours())
    