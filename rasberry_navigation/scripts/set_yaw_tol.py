# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 13:30:20 2019

@author: adam
"""
#####################################################################################
from utils import load_data_from_yaml, get_positions
import copy, yaml, numpy as np


write_map = True
polytunnel_wayPoints = False
set_xy_goal_tolerance = True
set_yaw_goal_tolerance = False
xy_goal_tolerance = 0.05
yaw_goal_tolerance = 6.28


base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme_bidirectional_sim.tmap"
outfile = "riseholme_bidirectional_sim_new.tmap"

# way points in polytunnels
cA = get_positions("centres_polytunnelA.txt")
cB = get_positions("centres_polytunnelB.txt")
wayPoints_in = np.ndarray.tolist(cA[:, 0])
wayPoints_in.extend(np.ndarray.tolist(cB[:, 0]))

# way points not in polytunnels
wayPoints_out = [63, 56, 66, 74, 73, 67, 68, 69, 70, 71, 72]

if polytunnel_wayPoints:
    wayPoints = wayPoints_in
else:
    wayPoints = wayPoints_out


f_in = base_dir + "/" + infile
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)

for i, way_point in enumerate(wayPoints):
    way_point_str = "WayPoint" + str(int(way_point))
    
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            
            if set_xy_goal_tolerance:
                cpy[j]["node"]["xy_goal_tolerance"] = xy_goal_tolerance            
            
            if set_yaw_goal_tolerance:
                cpy[j]["node"]["yaw_goal_tolerance"] = yaw_goal_tolerance
            
            
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
######################################################################################