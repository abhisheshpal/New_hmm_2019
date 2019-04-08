# -*- coding: utf-8 -*-
"""
Created on Wed Sep  5 09:57:43 2018

@author: Adam Binch

Adjust the positions of the polytunnel A nodes so that they are centred between 
the poles.
"""
#####################################################################################
import yaml, copy
from utils import load_data_from_yaml, get_positions
        
        
write_map = False

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme.tmap"
outfile = "riseholme_new.tmap"

f_in = base_dir + "/" + infile 
topo_map = load_data_from_yaml(f_in)

c = get_positions("centres_polytunnelA.txt")

way_points = c[:, 0]
centres = c[:, 1:3]

cpy = copy.deepcopy(topo_map)
for i, way_point in enumerate(way_points):
    way_point_str = "WayPoint" + str(int(way_point))
    
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            print "adjusting position of node " + way_point_str
            
            cpy[j]['node']['pose']['position']['x'] = float(centres[i][0])
            cpy[j]['node']['pose']['position']['y'] = float(centres[i][1])
            
            
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################