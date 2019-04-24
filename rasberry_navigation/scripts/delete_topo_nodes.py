# -*- coding: utf-8 -*-
"""
Created on Thu Sep 27 14:36:16 2018

@author: Adam Binch

Delete nodes specified in `way_points`.
"""
#####################################################################################
from __future__ import division
from utils import load_data_from_yaml
import yaml, copy, re


write_map = True
not_in = True 

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme_bidirectional.tmap"
outfile = "riseholme-uv_bidirectional.tmap"

way_points = [70,71,72,69,68]
way_points.extend([67,73,74,66,56,63])
way_points.extend([131,132,133,134,138,139])
way_points.extend([57,58,59,60,64,65])
way_points.extend([120,119,110,101,84,83])
way_points.extend([46,45,36,27,10,9])
#####################################################################################


#####################################################################################
way_point_strs = ["WayPoint" + str(int(way_point)) for way_point in way_points]
f_in = base_dir + "/" + infile  
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)
regex = "(.*)_(.*)"

indices = []
for i, node in enumerate(cpy):
    if not_in:       
        if node["meta"]["node"] not in way_point_strs:
            indices.append(i)
    else:
        if node["meta"]["node"] in way_point_strs:
            indices.append(i)
            
for i in sorted(indices, reverse=True):
    print "deleting node " + cpy[i]["meta"]["node"]
    del cpy[i]    
    

# Delete edges with destinations that no longer exist
remaining_nodes = [node["meta"]["node"] for node in cpy]
for i, node in enumerate(cpy):
    edges = copy.deepcopy(cpy[i]["node"]["edges"])
    new_edges = []
    for edge in edges:
        destination = re.match(regex, edge["edge_id"]).groups()[1]
        if destination in remaining_nodes:
            new_edges.append(edge)
    cpy[i]["node"]["edges"] = new_edges

         
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################        