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
#base_dir = rospack.get_path('rasberry_gazebo')
base_dir = os.getcwd()[:-8]

world_f = base_dir + '/worlds/empty_grass.world'
world_d = load_data_from_xml(world_f)
world = AddtoWorld(world_d)
world_name = 'thorvald.world' # default

default_model_config = base_dir + '/config/gazebo/models_AB.yaml'
default_actor_config = base_dir + '/config/gazebo/actors_AB.yaml'

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
        pot21_f = base_dir + '/models/pot21/model.sdf'
        pot9_f = base_dir + '/models/pot9/model.sdf'
        pot1_f = base_dir + '/models/pot1/model.sdf'
        #plant_f = base_dir + '/models/plant/model.sdf'
        plant21_f = base_dir + '/models/plant21/model.sdf'
        plant9_f = base_dir + '/models/plant9/model.sdf'
        plant1_f = base_dir + '/models/plant1/model.sdf'
        arch_f = base_dir + '/models/dummy_arch/model.sdf'
        strut_f = base_dir + '/models/strut/model.sdf'
        canopy10m_f = base_dir + '/models/canopy10m/model.sdf'
        canopy4m_f = base_dir + '/models/canopy4m/model.sdf'
        canopyhalfm_f = base_dir + '/models/canopyhalfm/model.sdf'
        
        
        pole_d = load_data_from_xml(pole_f)    
        tray_d = load_data_from_xml(tray_f)    
        trayp2_d = load_data_from_xml(trayp2_f)
        pot21_d = load_data_from_xml(pot21_f)
        pot9_d = load_data_from_xml(pot9_f)
        pot1_d = load_data_from_xml(pot1_f)
        #plant_d = load_data_from_xml(plant_f)
        plant21_d = load_data_from_xml(plant21_f)
        plant9_d = load_data_from_xml(plant9_f)
        plant1_d = load_data_from_xml(plant1_f)
        arch_d = load_data_from_xml(arch_f)
        strut_d = load_data_from_xml(strut_f) 
        canopy10m_d = load_data_from_xml(canopy10m_f)        
        canopy4m_d = load_data_from_xml(canopy4m_f)        
        canopyhalfm_d = load_data_from_xml(canopyhalfm_f) 
        
        
        pole_count = 0
        tray_count = 0
        trayp2_count = 0
        pot21_count = 0
        pot9_count = 0
        pot1_count = 0
        #plant_count = 0
        plant21_count = 0
        plant9_count = 0
        plant1_count = 0
        arch_count = 0
        strut_count = 0
        canopy10m_count = 0   
        canopy4m_count = 0   
        canopyhalfm_count = 0
        
        
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
                
                strut_positions = polytunnels[i]['strut_positions']
                
                
            
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
                    
                    pot21_poses, pot21_max_length = get_pot21_poses([pole_xposes[0], pole_xposes[-1]], pole_yposes)
                    #world.add_pot21(pot21_d, pot21_poses, pot21_count)  
                    pot21_count += len(pot21_poses)
                
                    pot9_poses, pot9_max_length = get_pot9_poses([pole_xposes[0], pole_xposes[-1]], pole_yposes, pot21_max_length)
                    #world.add_pot9(pot9_d, pot9_poses, pot9_count)  
                    pot9_count += len(pot9_poses)
                    
                    pot1_poses, pot1_xposes = get_pot1_poses([pole_xposes[0], pole_xposes[-1]], pole_yposes, pot21_max_length, pot9_max_length) 
                    #world.add_pot1(pot1_d, pot1_poses, pot1_count)
                    pot1_count += len(pot1_poses)
                
                    #plant_poses = get_plant_poses(pot1_xposes, pole_yposes)
                    #world.add_plants(plant_d, plant_poses, plant_count)  
                    #plant_count += len(plant_poses)
                    
                    #world.add_plant21(plant21_d, pot21_poses, plant21_count)  
                    plant21_count += len(pot21_poses)                    
                    
                    #world.add_plant9(plant9_d, pot9_poses, plant9_count)  
                    plant9_count += len(pot9_poses)
                    
                    #world.add_plant1(plant1_d, pot1_poses, plant1_count)  
                    plant1_count += len(pot1_poses)


                if (arch_nx * arch_dx) > 0:
                    arch_poses, arch_xposes, arch_yposes = get_arch_poses(arch_nx, arch_dx, arch_xoffset, pole_dy, pole_yposes)
                    world.add_arches(arch_d, arch_poses, arch_count)
                    arch_count += len(arch_poses)   
                    
                    canopy10m_poses, canopy10m_max_length = get_canopy10m_poses([arch_xposes[0], arch_xposes[-1]], arch_yposes[0], pole_dy)
                    world.add_canopy10m(canopy10m_d, canopy10m_poses, canopy10m_count)
                    canopy10m_count += len(canopy10m_poses)
                    
                    canopy4m_poses, canopy4m_max_length = get_canopy4m_poses([arch_xposes[0], arch_xposes[-1]], arch_yposes[0], pole_dy, canopy10m_max_length)
                    world.add_canopy4m(canopy4m_d, canopy4m_poses, canopy4m_count)
                    canopy4m_count += len(canopy4m_poses)

                    canopyhalfm_poses = get_canopyhalfm_poses([arch_xposes[0], arch_xposes[-1]], arch_yposes[0], pole_dy, canopy10m_max_length, canopy4m_max_length)
                    world.add_canopyhalfm(canopyhalfm_d, canopyhalfm_poses, canopyhalfm_count)
                    canopyhalfm_count += len(canopyhalfm_poses) 
                    
                    for position in strut_positions:
                        if position == "L":
                            strut_poses, strut_xposes, strut_yposes, strut_roll = get_strut_poses_L(arch_xposes, arch_yposes, arch_dx, pole_ny, pole_dy)
                        if position == "R":
                            strut_poses, strut_xposes, strut_yposes, strut_roll = get_strut_poses_R(arch_xposes, arch_yposes, arch_dx)                        
                    
                        world.add_struts(strut_d, strut_poses, strut_roll, strut_count)
                        strut_count += len(strut_poses)           
                    
   


    if model.keys()[0] == 'frontage':
        frontage_f = base_dir + '/models/frontage/model.sdf'    
        frontage_d = load_data_from_xml(frontage_f)
        
        frontage = model['frontage']
        for i in range(len(frontage)):
            if frontage[i]['include']:
                
                xpose = frontage[i]['xpose']
                ypose = frontage[i]['ypose']
                world.add_frontage(frontage_d, [xpose, ypose], i)   
   
   
    if model.keys()[0] == 'riseholme_enclosure':
        riseholme_enclosure_f = base_dir + '/models/riseholme_enclosure/model.sdf'    
        riseholme_enclosure_d = load_data_from_xml(riseholme_enclosure_f)
        
        riseholme_enclosure = model['riseholme_enclosure']
        for i in range(len(riseholme_enclosure)):
            if riseholme_enclosure[i]['include']:
                
                pose = riseholme_enclosure[i]['pose']
                world.add_riseholme_enclosure(riseholme_enclosure_d, pose, i)
                
                
                
    if model.keys()[0] == 'food_handling_unit':
        fhs_f = base_dir + '/models/food_handling_unit/model.sdf'  
        fhs_floor_f = base_dir + '/models/fhu_floor/model.sdf' 
        fhs_d = load_data_from_xml(fhs_f)
        fhs_floor_d = load_data_from_xml(fhs_floor_f)
        
        fhs = model['food_handling_unit']
        for i in range(len(fhs)):
            if fhs[i]['include']:
                
                pose = fhs[i]['pose']
                world.add_fhs(fhs_d, pose, i)
                world.add_fhs_floor(fhs_floor_d, pose, i)


    if model.keys()[0] == 'lab':
        lab_f = base_dir + '/models/lab/model.sdf'  
        lab_floor_f = base_dir + '/models/lab_floor/model.sdf' 
        lab_d = load_data_from_xml(lab_f)
        lab_floor_d = load_data_from_xml(lab_floor_f)
        
        lab = model['lab']
        for i in range(len(lab)):
            if fhs[i]['include']:
                
                pose = lab[i]['pose']
                world.add_lab(lab_d, pose, i)
                world.add_lab_floor(lab_floor_d, pose, i)                
                            
                            
    if model.keys()[0] == 'world_name':  
        world_name = model['world_name']              
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