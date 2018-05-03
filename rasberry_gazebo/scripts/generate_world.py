#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 14 11:36:00 2018

@author: Adam Binch
@email: adambinch@gmail.com
"""
################################################################################
import argparse, xmltodict, os, rospkg
from add_to_world import AddtoWorld
from utils import load_data_from_yaml, load_data_from_xml
from generate_poses import *


save_world = True

rospack = rospkg.RosPack()
base_dir = rospack.get_path('rasberry_gazebo')
#base_dir = os.getcwd()[:-8]

world_f = base_dir + '/worlds/empty_grass.world'
world_d = load_data_from_xml(world_f)
world = AddtoWorld(world_d)

default_model_config = base_dir + '/config/models_AB.yaml'
default_actor_config = base_dir + '/config/actors_AB.yaml'

parser = argparse.ArgumentParser()
parser.add_argument("--model_file", type=str, default=default_model_config, help="filename")       
parser.add_argument("--actor_file", type=str, default=default_actor_config, help="filename")
args = parser.parse_args()
model_config = load_data_from_yaml(args.model_file)
actor_config = load_data_from_yaml(args.actor_file)
################################################################################


################################################################################
# MODELS


for model in model_config:
    
    if model.keys()[0] == 'polytunnel':
        pole_f = base_dir + '/models/pole/model.sdf'
        tray_f = base_dir + '/models/tray/model.sdf'
        trayp2_f = base_dir + '/models/tray_part2/model.sdf'
        arch_f = base_dir + '/models/dummy_arch/model.sdf'
        canopy_f = base_dir + '/models/canopy/model.sdf'
        pot_f = base_dir + '/models/pot/model.sdf'
        plant_f = base_dir + '/models/plant/model.sdf'
        plant2_f = base_dir + '/models/plant2/model.sdf'
        
        
        pole_d = load_data_from_xml(pole_f)    
        tray_d = load_data_from_xml(tray_f)    
        trayp2_d = load_data_from_xml(trayp2_f)
        arch_d = load_data_from_xml(arch_f)
        canopy_d = load_data_from_xml(canopy_f)
        pot_d = load_data_from_xml(pot_f)
        plant_d = load_data_from_xml(plant_f)
        plant2_d = load_data_from_xml(plant2_f)
        
        
        pole_count = 0
        tray_count = 0
        trayp2_count = 0  
        pot_count = 0
        plant_count = 0
        plant2_count = 0
        arch_count = 0
        canopy_count = 0   
        
        
        polytunnels = model['polytunnel']
        for i in range(len(polytunnels)):
            if polytunnels[i]['include']:            
            
                pole_nx = polytunnels[i]['pole_nx']
                pole_ny = polytunnels[i]['pole_ny']
                pole_dx = polytunnels[i]['pole_dx']
                pole_dy = polytunnels[i]['pole_dy']
                pole_xoffset = polytunnels[i]['pole_xoffset']
                pole_yoffset = polytunnels[i]['pole_yoffset']
                
                arch_nx = polytunnels[i]['arch_nx']
                arch_dx = polytunnels[i]['arch_dx']
                arch_xoffset = polytunnels[i]['arch_xoffset']
                
                
            
                if (pole_nx * pole_dx) > 0 and (pole_ny * pole_dy) > 0:
                    pole_poses, pole_xposes, pole_yposes = get_pole_poses(pole_nx, pole_ny, pole_dx, pole_dy, pole_xoffset, pole_yoffset) 
                    world.add_poles(pole_d, pole_poses, pole_count)
                    pole_count += len(pole_poses)
                
                    tray_poses, tray_length = get_tray_poses(pole_nx, pole_ny, pole_dx, pole_xoffset, pole_yposes)
                    world.add_trays(tray_d, tray_poses, tray_length, tray_count) 
                    tray_count += len(tray_poses)
                
                    trayp2_poses = get_trayp2_poses(tray_length, pole_xoffset, pole_ny, pole_yposes)
                    world.add_traysp2(trayp2_d, trayp2_poses, tray_length, trayp2_count)                 
                    trayp2_count += len(trayp2_poses)
                
                    pot_poses, pot_xposes = get_pot_poses([pole_xposes[0], pole_xposes[-1]], pole_yposes) 
                    world.add_pots(pot_d, pot_poses, pot_count)
                    pot_count += len(pot_poses)
                
                    #plant_poses = get_plant_poses(pot_xposes, pole_yposes)
                    #world.add_plants(plant_d, plant_poses, plant_count)  
                    #plant_count += len(plant_poses)
                    
                    world.add_plants2(plant2_d, pot_poses, plant2_count)  
                    plant2_count += len(pot_poses)


                if (arch_nx * arch_dx) > 0:
                    arch_poses, arch_xposes, arch_yposes = get_arch_poses(arch_nx, arch_dx, arch_xoffset, pole_dy, pole_yposes)
                    world.add_arches(arch_d, arch_poses, arch_count)
                    arch_count += len(arch_poses)                    
                    
                    canopy_poses, canopy_xposes, canopy_yposes = get_canopy_poses([arch_xposes[0], arch_xposes[-1]], arch_yposes[0], pole_dy)
                    world.add_canopy(canopy_d, canopy_poses, canopy_count)
                    canopy_count += len(canopy_poses)
    
   
   
    if model.keys()[0] == 'riseholme_enclosure':
        riseholme_enclosure_f = base_dir + '/models/riseholme_enclosure/model.sdf'    
        riseholme_enclosure_d = load_data_from_xml(riseholme_enclosure_f)
        
        riseholme_enclosure = model['riseholme_enclosure']
        for i in range(len(riseholme_enclosure)):
            if riseholme_enclosure[i]['include']:
                
                xpose = riseholme_enclosure[i]['xpose']
                ypose = riseholme_enclosure[i]['ypose']
                world.add_riseholme_enclosure(riseholme_enclosure_d, [xpose, ypose], i)
                
                
                
    if model.keys()[0] == 'food_handling_shed':
        fhs_f = base_dir + '/models/food_handling_shed/model.sdf'  
        fhs_floor_f = base_dir + '/models/fhs_floor/model.sdf' 
        fhs_d = load_data_from_xml(fhs_f)
        fhs_floor_d = load_data_from_xml(fhs_floor_f)
        
        fhs = model['food_handling_shed']
        for i in range(len(fhs)):
            if fhs[i]['include']:
                
                xpose = fhs[i]['xpose']
                ypose = fhs[i]['ypose']
                world.add_fhs(fhs_d, [xpose, ypose], i)
                world.add_fhs_floor(fhs_floor_d, [xpose, ypose], i)
                
                            
                            
    if model.keys()[0] == 'world_name':  
        world_name = model['world_name']              
    else:
        world_name = 'thorvald_AB.world' # default
################################################################################



################################################################################        
# ACTORS


actor_f = base_dir + '/scripts/actor'
actor_d = load_data_from_xml(actor_f)
for actor in actor_config:
    if actor['include']:
        world.add_actors(actor_d, actor)
################################################################################



################################################################################        
m = world.world_d['sdf']['world']['model']
for i in range(len(m)):
    print "-----------------"
    print "[%s]"%m[i]['@name']
    
a = world.world_d['sdf']['world']['actor']
for i in range(len(a)):
    print "-----------------"
    print "[%s]"%a[i]['@name']    

if save_world:
    fh = open(base_dir + '/worlds/' + world_name, "w")
    s_output = xmltodict.unparse(world.world_d, pretty=True)
    fh.write(s_output)
    fh.close()    
################################################################################
