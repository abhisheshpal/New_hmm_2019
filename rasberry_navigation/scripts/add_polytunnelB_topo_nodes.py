# -*- coding: utf-8 -*-
"""
Created on Wed Oct 17 08:45:22 2018

@author: Adam Binch

This script adds the nodes and edges for polytinnel B to the topological map by 
re-using those for polytunnel A. Note that this script adds edges that occur 
within the polytunnel only. All other edges were added using 
`rasberry_navigation/scripts/add_edges_between_nodes.py` 
"""
#####################################################################################
import yaml, copy, re
from utils import load_data_from_yaml, get_positions


write_map = False

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme.tmap"
outfile = "riseholme_new.tmap"

cA = get_positions("centres_polytunnelA.txt")
wayPoints_pTunnelA = cA[:, 0]

cB = get_positions("centres_polytunnelB.txt")
wayPoints_pTunnelB = cB[:, 0]
centres_pTunnelB = cB[:, 1:3]


f_in = base_dir + "/" + infile        
topo_map = load_data_from_yaml(f_in)

cpy = copy.deepcopy(topo_map)
indices = []
for way_point in wayPoints_pTunnelA:
    way_point_str = "WayPoint" + str(int(way_point))
    
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            indices.append(j)
            

x = [66, 56, 63]
regex = "(.*)_(.*)"
for i, index in enumerate(indices):
    node = copy.deepcopy(cpy[index])
    
    new_origin = "WayPoint{}".format(int(wayPoints_pTunnelB[i]))
    
    node["meta"]["node"] = new_origin
    node["node"]["name"] = new_origin
    
    node['node']['pose']['position']['x'] = float(centres_pTunnelB[i][0])
    node['node']['pose']['position']['y'] = float(centres_pTunnelB[i][1])
    
    edges_pTunnelA = copy.deepcopy(node["node"]["edges"])
    edges_pTunnelB = []
    for j, edge in enumerate(edges_pTunnelA):
        edge_id = copy.deepcopy(edge["edge_id"])
        
        m_regex = re.match(regex, edge_id)
        destination_num = m_regex.groups()[1][8:]
        
        if int(destination_num) in x:
            continue
        
        else:
            new_destination = "WayPoint{}".format(int(destination_num) + 74)
            new_edge_id = new_origin + "_" + new_destination    
        
            edge["node"] = new_destination
            edge["edge_id"] = new_edge_id
            edges_pTunnelB.append(edge)

    node["node"]["edges"] = edges_pTunnelB
    cpy.append(node)
    
    
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################
