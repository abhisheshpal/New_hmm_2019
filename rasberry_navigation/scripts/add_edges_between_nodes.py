# -*- coding: utf-8 -*-
"""
Created on Wed Oct 17 15:50:32 2018

@author: Adam Binch

Add edges specified in `e`. The left number is the origin, the right is 
the destination.
"""
#####################################################################################
import yaml, copy
from utils import load_data_from_yaml


write_map = True

base_dir = "/home/adam/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme-uv_poly_act_sim_2.tmap"
outfile = "riseholme-uv_poly_act_sim_3.tmap"

edge_action = "row_traversal"
edges = [[131,120],[120,131],[132,119],[119,132],[133,110]]
edges.extend([[110,133],[134,101],[101,134],[138,84],[84,138]])
edges.extend([[57,46],[46,57],[58,45],[45,58],[59,36],[36,59]])
edges.extend([[60,27],[27,60],[64,10],[10,64]])


f_in = base_dir + "/" + infile        
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)
for edge in edges:    
    origin = "WayPoint{}".format(edge[0])
    
    for i, node in enumerate(cpy):
        if node["node"]["name"] == origin:
            destination = "WayPoint{}".format(edge[1])
            edge_id = origin + "_" + destination 
            
            print "adding edge " + edge_id
            
            new_edge = {'action':   edge_action,
                        'edge_id': edge_id,
                        'inflation_radius': 0.0,
                        'map_2d': 'riseholme',
                        'node': destination,
                        'recovery_behaviours_config': '',
                        'top_vel': 0.55}
                        
            cpy[i]["node"]["edges"].append(new_edge)

    
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################