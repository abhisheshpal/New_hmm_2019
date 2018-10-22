# -*- coding: utf-8 -*-
"""
Created on Thu Sep 27 14:36:16 2018

@author: Adam Binch

Delete nodes specified in `way_points`.
"""
#####################################################################################
from __future__ import division
from utils import load_data_from_yaml
import yaml, copy


write_map = False

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme_sim.tmap"
outfile = "riseholme-uv_sim.tmap"

way_points = [6,2,1,5,4,3,7,8]
way_points.extend([18,17,16,15,14,13,12,11])
way_points.extend([19,20,21,22,23,24,25,26])
way_points.extend([28,29,30,31,32,33,34,35])
way_points.extend([37,38,39,40,41,42,43,44])
way_points.extend([54,53,52,51,50,49,48,47])


f_in = base_dir + "/" + infile  
topo_map = load_data_from_yaml(f_in)

indices = []
cpy = copy.deepcopy(topo_map)
for i, way_point in enumerate(way_points):
    way_point_str = "WayPoint" + str(int(way_point))
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            indices.append(j)
            
for i in sorted(indices, reverse=True):
    del cpy[i]            

if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################        