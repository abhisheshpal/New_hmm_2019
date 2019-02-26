# -*- coding: utf-8 -*-
"""
Created on Wed Oct 17 15:50:32 2018

@author: Pratik Somaiya

Note: use it to convert edges from unidrectional to bidirectional
"""
#####################################################################################
import yaml, copy, rospkg
from utils import load_data_from_yaml

write_map = True
node_pairs = []
j=0

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rasberry_navigation
base_dir = rospack.get_path('rasberry_navigation')
infile = "maps/riseholme.tmap"
outfile = "maps/riseholme_new.tmap"


f_in = base_dir + "/" + infile        
topo_map = load_data_from_yaml(f_in)
cpy = copy.deepcopy(topo_map)

for i, node in enumerate(cpy):
    source = node["node"]["name"]
    for k in range(len(node["node"]["edges"])):
	    destination = node["node"]["edges"][k]["node"]
	    node_pairs.append((source,destination))


while j!=len(node_pairs):
    for i, node in enumerate(cpy):
        if node["node"]["name"] == node_pairs[j][1]:
            destination = node_pairs[j][0]
            edge_id = node_pairs[j][1] + "_" + node_pairs[j][0] 
            print "adding edge " + edge_id
            
            new_edge = {'action': 'move_base',
                        'edge_id': edge_id,
                        'inflation_radius': 0.0,
                        'map_2d': 'riseholme',
                        'node': destination,
                        'recovery_behaviours_config': '',
                        'top_vel': 0.55}
                        
            cpy[i]["node"]["edges"].append(new_edge)
            j+=1
            if (j==len(node_pairs)):
                break;

    
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################
