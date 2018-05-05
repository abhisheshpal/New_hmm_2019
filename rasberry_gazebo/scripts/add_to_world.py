# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 13:55:50 2018

@author: Adam Binch
@email: adambinch@gmail.com
"""
################################################################################
import copy


class AddtoWorld(object):
    
    def __init__(self, world_d):
        
        world_d['sdf']['world']['model']=[] # creates a list to store the models
        world_d['sdf']['world']['actor']=[] # creates a list to store the actors
        self.world_d = world_d
        
        
    def add_poles(self, pole_d, pole_poses, pole_count=0):
        n = len(pole_poses)
        for i in range(n):
            pole_id = pole_count+i
            cpy= pole_d['sdf']['model']
            cpy['@name'] = u'pole_' + str(pole_id)
            posstr= str("%.3f %.3f 0.691 0.0 0.0 0.0"%(pole_poses[i][0], pole_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))      


    def add_trays(self, tray_d, tray_poses, tray_length, tray_count=0):
        n = len(tray_poses)
        for i in range(n):
            tray_id = tray_count+i
            cpy= tray_d['sdf']['model']
            cpy['@name'] = u'tray_' + str(tray_id)
            posstr = str("%.3f %.3f 1.382 0.0 0.0 0.0"%(tray_poses[i][0], tray_poses[i][1]))
            cpy['link']['pose']['#text'] = posstr
            geomstr = str("%.3f 0.15 0.005"%(tray_length))
            cpy['link']['collision']['geometry']['box']['size'] = geomstr
            cpy['link']['visual']['geometry']['box']['size'] = geomstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
            
    def add_traysp2(self, trayp2_d, trayp2_poses, tray_length, trayp2_count=0):
        n = len(trayp2_poses)
        for i in range(n):
            trayp2_id = trayp2_count+i
            cpy= trayp2_d['sdf']['model']
            cpy['@name'] = u'tray_p2_' + str(trayp2_id)
            posstr = str("%.3f %.3f 1.424 1.5707 0.0 0.0"%(trayp2_poses[i][0], trayp2_poses[i][1]))
            cpy['link']['pose']['#text'] = posstr
            geomstr = str("%.2f 0.085 0.005"%(tray_length))
            cpy['link']['collision']['geometry']['box']['size'] = geomstr
            cpy['link']['visual']['geometry']['box']['size'] = geomstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
            
    def add_pots(self, pot_d, pot_poses, pot_count=0):
        n = len(pot_poses)
        for i in range(n):
            pot_id = pot_count+i
            cpy= pot_d['sdf']['model']
            cpy['@name'] = u'pot_' + str(pot_id)
            posstr = str("%.3f %.3f 1.467 0.0 0.0 0.0"%(pot_poses[i][0], pot_poses[i][1]))
            cpy['link']['pose']['#text'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
            
    def add_pot_arrays(self, pot_array_d, pot_array_poses, pot_array_count=0):
        n = len(pot_array_poses)
        for i in range(n):
            pot_array_id = pot_array_count+i
            cpy= pot_array_d['sdf']['model']
            cpy['@name'] = u'pot_array_' + str(pot_array_id)
            posstr = str("%.3f %.3f 1.382 1.5707 0.0 1.5707"%(pot_array_poses[i][0], pot_array_poses[i][1]))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))           
            
            
    def add_plants(self, plant_d, plant_poses, plant_count=0):
        n = len(plant_poses)
        for i in range(n):
            plant_id = plant_count+i
            cpy= plant_d['sdf']['model']
            cpy['@name'] = u'plant_' + str(plant_id)
            posstr= str("%.3f %.3f %.3f 0.0 0.0 %.4f"%(plant_poses[i][0], plant_poses[i][1], 
                                                       plant_poses[i][2], plant_poses[i][3]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))                  
            
            
    def add_plants2(self, plant2_d, plant2_poses, plant2_count=0):
        n = len(plant2_poses)
        for i in range(n):
            plant2_id = plant2_count+i
            cpy= plant2_d['sdf']['model']
            cpy['@name'] = u'plant2_' + str(plant2_id)
            posstr = str("%.3f %.3f 1.467 1.5707 0.0 0.0"%(plant2_poses[i][0], plant2_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))


    def add_plant_array(self, plant_array_d, plant_array_poses, plant_array_count=0):
        n = len(plant_array_poses)
        for i in range(n):
            plant_array_id = plant_array_count+i
            cpy= plant_array_d['sdf']['model']
            cpy['@name'] = u'plant_array_' + str(plant_array_id)
            posstr = str("%.3f %.3f 1.502 1.5707 0.0 1.5707"%(plant_array_poses[i][0]-0.15, plant_array_poses[i][1]-0.15))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))                             
            

    def add_arches(self, arch_d, arch_poses, arch_count=0):
        n = len(arch_poses)
        for i in range(n):
            arch_id = arch_count+i
            cpy= arch_d['sdf']['model']
            cpy['@name'] = u'dummy_arch_' + str(arch_id)
            posstr= str("%.3f %.3f 0.0 1.5707 0.0 1.5707"%(arch_poses[i][0], arch_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            

    def add_canopy(self, canopy_d, canopy_poses, canopy_count=0):
        n = len(canopy_poses)
        for i in range(n):
            canopy_id = canopy_count+i
            cpy= canopy_d['sdf']['model']
            cpy['@name'] = u'canopy_' + str(canopy_id)
            posstr= str("%.3f %.3f 0.0 1.5707 0.0 1.5707"%(canopy_poses[i][0], canopy_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))  
            
            
    def add_canopy_small(self, canopy_small_d, canopy_small_poses, canopy_small_count=0):
        n = len(canopy_small_poses)
        for i in range(n):
            canopy_small_id = canopy_small_count+i
            cpy= canopy_small_d['sdf']['model']
            cpy['@name'] = u'canopy_small_' + str(canopy_small_id)
            posstr= str("%.3f %.3f 0.0 1.5707 0.0 1.5707"%(canopy_small_poses[i][0], canopy_small_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
            
            
    def add_riseholme_enclosure(self, riseholme_enclosure_d, pose, j=0):
        cpy= riseholme_enclosure_d['sdf']['model']
        cpy['@name'] = u'riseholme_enclosure_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 0.0"%(pose[0], pose[1]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
        
        
    def add_fhs(self, fhs_d, pose, j=0):
        cpy= fhs_d['sdf']['model']
        cpy['@name'] = u'food_handling_shed_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 0.0"%(pose[0], pose[1]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
        
        
    def add_fhs_floor(self, fhs_floor_d, pose, j=0):
        cpy= fhs_floor_d['sdf']['model']
        cpy['@name'] = u'fhs_floor_' + str(j)
        posstr= str("%.3f %.3f 0.01 0.0 0.0 0.0"%(pose[0], pose[1]))
        cpy['link']['pose']['#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
        
    
    def add_actors(self, actor_d, actor):    
        cpy_actor = copy.deepcopy(actor_d['actor'])
        cpy_actor['@name'] = actor['name']
        cpy_actor['animation']['filename'] = actor['animation']
        cpy_actor['script']['trajectory']['waypoint'] = []
        for waypoint in actor['waypoints']:
            d = {'pose': '', 'time': ''}
            d['time'] = waypoint['time']
            d['pose'] = waypoint['pose'] 
            cpy_actor['script']['trajectory']['waypoint'].append(copy.deepcopy(d))    
        self.world_d['sdf']['world']['actor'].append(copy.deepcopy(cpy_actor))        
################################################################################            
