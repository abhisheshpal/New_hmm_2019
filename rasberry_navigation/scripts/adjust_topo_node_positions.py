# -*- coding: utf-8 -*-
"""
Created on Wed Oct 17 15:29:38 2018

@author: Adam Binch

Shift positions of nodes specified in `way_points` by dx, dy.
"""
#####################################################################################
from utils import load_data_from_yaml
import yaml, copy


write_map = False

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme.tmap"
outfile = "riseholme_new.tmap"

way_points = [120,119,110,101,84,83]
dx = 0.5
dy = -0.04

f_in = base_dir + "/" + infile
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)
for i, way_point in enumerate(way_points):
    way_point_str = "WayPoint" + str(int(way_point))
    
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            print "adjusting position of node " + way_point_str            
            
            x = copy.deepcopy(cpy[j]['node']['pose']['position']['x'])
            y = copy.deepcopy(cpy[j]['node']['pose']['position']['y'])
            
            cpy[j]['node']['pose']['position']['x'] = float(x+dx)
            cpy[j]['node']['pose']['position']['y'] = float(y+dy)

if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
######################################################################################        