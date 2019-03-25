#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.farm


class FarmMimic(rasberry_des.farm.Farm):
    """Farm class definition"""
    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose=False):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        n_topo_nav_rows -- navigational rows in topological map
        topo_map -- TopologicalForkMap object
        robots - robot agent objects
        pickers -- picker agent objects
        policy -- "lexicographical", "shortest_distance", "uniform_utilisation"
        """
        super(FarmMimic, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)