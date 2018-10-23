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


write_map = False

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme.tmap"
outfile = "riseholme_new.tmap"

e = [[67,131],[67,132],[67,133],[67,134],[67,138],[67,139]]
e.extend([[73,131],[73,132],[73,133],[73,134],[73,138],[73,139]])
e.extend([[74,131],[74,132],[74,133],[74,134],[74,138],[74,139]])
e.extend([[131,67],[131,73],[131,74]])
e.extend([[132,67],[132,73],[132,74]])
e.extend([[133,67],[133,73],[133,74]])
e.extend([[134,67],[134,73],[134,74]])
e.extend([[138,67],[138,73],[138,74]])
e.extend([[139,67],[139,73],[139,74]])


f_in = base_dir + "/" + infile        
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)
for edge in e:    
    origin = "WayPoint{}".format(edge[0])
    
    for i, node in enumerate(cpy):
        if node["node"]["name"] == origin:
            destination = "WayPoint{}".format(edge[1])
            edge_id = origin + "_" + destination 
            
            print "adding edge " + edge_id
            
            new_edge = {'action': 'move_base',
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