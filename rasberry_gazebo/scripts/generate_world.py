#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 14 11:36:00 2018

@author: adam
"""
################################################################################
from __future__ import division
import numpy as np
from itertools import product
import json, xmltodict, yaml, argparse
from add_to_world import AddtoWorld


def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.load(f)


def load_data_from_xml(filename):
    with open(filename) as f:
        doc = json.dumps(xmltodict.parse(f.read()))
        return json.loads(doc)        
    
    
def get_pole_poses(nx, ny, dx, dy, xoffset, yoffset):
    
    xposes = np.arange(0, nx*dx, dx) + xoffset
    yposes = np.arange(0, ny*dy, dy) + yoffset
    
    poses = []
    for i, j in list(product(range(nx), range(ny))):
        pose = [xposes[i], yposes[j]]
        poses.append(pose)
    return poses, xposes, yposes
################################################################################    


################################################################################
save_world = True

base_dir = '/home/adam/rasberry_ws/src/RASberry/rasberry_gazebo'
default_config = base_dir + '/config/test.yaml'

parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str, default=default_config, help="filename")
args = parser.parse_args()
model_config = load_data_from_yaml(args.file)

world_f = base_dir + '/worlds/empty.world'
world_d = load_data_from_xml(world_f)
world = AddtoWorld(world_d)

for model in model_config:
    
    if model.keys()[0] == 'polytunnel':
        pole_f = base_dir + '/models/unit_cylinder_1/model.sdf'
        tray_f = base_dir + '/models/tray/model.sdf'
        arch_f = base_dir + '/models/dummy_arch/model.sdf'
        
        pole_d = load_data_from_xml(pole_f)    
        tray_d = load_data_from_xml(tray_f)    
        arch_d = load_data_from_xml(arch_f)
        
        polytunnels = model['polytunnel']        
        for i in range(len(polytunnels)):
            nx = polytunnels[i]['nx']
            ny = polytunnels[i]['ny']
            dx = polytunnels[i]['dx']
            dy = polytunnels[i]['dy']
            xoffset = polytunnels[i]['xpose']
            yoffset = polytunnels[i]['ypose']
        
            pole_poses, pole_xposes, pole_yposes = get_pole_poses(nx, ny, dx, dy, xoffset, yoffset) 
            
            tray_length = (nx-1) * dx
            tray_xpose = tray_length/2 + xoffset
            tray_xposes = np.zeros(ny) + tray_xpose
            tray_poses = np.ndarray.tolist(np.vstack((tray_xposes, pole_yposes)).T) 
            
            arch_yposes = np.zeros(nx) + pole_yposes[0] - dy
            arch_poses = np.ndarray.tolist(np.vstack((pole_xposes, arch_yposes)).T)
            
            world.add_poles(pole_d, pole_poses, i)
            world.add_trays(tray_d, tray_poses, tray_length, i) 
            world.add_arches(arch_d, arch_poses, i)  

    if model.keys()[0] == 'riseholme_enclosure':

        riseholme_enclosure_f = base_dir + '/models/riseholme_enclosure/model.sdf'    
        riseholme_enclosure_d = load_data_from_xml(riseholme_enclosure_f)
        
        riseholme_enclosure = model['riseholme_enclosure']
        for i in range(len(riseholme_enclosure)):
            xpose = riseholme_enclosure[i]['xpose']
            ypose = riseholme_enclosure[i]['ypose']
            world.add_riseholme_enclosure(riseholme_enclosure_d, [xpose, ypose], i)
        
  
models = world.world_d['sdf']['world']['model']
for i in range(len(models)):
    print "-----------------"
    print "[%s]"%models[i]['@name']

if save_world:
    fh = open(base_dir + '/worlds/thorvald_AB.world', "w")
    s_output = xmltodict.unparse(world.world_d, pretty=True)
    fh.write(s_output)
    fh.close()    
#################################################################################