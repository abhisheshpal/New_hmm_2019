# -*- coding: utf-8 -*-
"""
Created on Wed Sep  5 09:57:43 2018

@author: Adam Binch

Adjust influence zones of nodes specified in `way_points` by delta. This script is 
currently set up to move the y-positions of the vertices only.
"""
#####################################################################################
import yaml, copy
from utils import load_data_from_yaml

      
write_map = False

delta = 0.2

base_dir = "/home/adam/workspaces/rasberry_ws/src/RASberry/rasberry_navigation/maps"
infile = "riseholme.tmap"
outfile = "riseholme_new.tmap"

way_points = [65,6,2,1,5,4,3,7,8,9]
way_points.extend([64,18,17,16,15,14,13,12,11,10])
way_points.extend([60,19,20,21,22,23,24,25,26,27])
way_points.extend([59,28,29,30,31,32,33,34,35,36])
way_points.extend([58,37,38,39,40,41,42,43,44,45])
way_points.extend([57,54,53,52,51,50,49,48,47,46])


f_in = base_dir + "/" + infile        
topo_map = load_data_from_yaml(f_in)

cpy = copy.deepcopy(topo_map)
verts = copy.deepcopy(cpy[0]['node']['verts'])
for i, way_point in enumerate(way_points):
    way_point_str = "WayPoint" + str(int(way_point))
    
    for j, node in enumerate(cpy):
        if node["node"]["name"] == way_point_str:
            print "adjusting infleunce zone of node " + way_point_str
            
            verts = copy.deepcopy(cpy[j]['node']['verts'])
            for k, vert in enumerate(verts):
                
                if k == 1 or k == 2 or k == 5 or k == 6: # the verts that you want to change

                    if vert['y'] > 0:
                        vert['y'] -= delta
                    else:
                        vert['y'] += delta                
            
            cpy[j]['node']['verts'] = verts
            
            
if write_map:
    with open(base_dir + "/" + outfile, 'w') as f_out:
        yaml.dump(cpy, f_out, default_flow_style=False)
#####################################################################################
