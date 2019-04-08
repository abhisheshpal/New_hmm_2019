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
            posstr= str("%.3f %.3f 0.575 0.0 0.0 0.0"%(pole_poses[i][0], pole_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))      


    def add_trays(self, tray_d, tray_poses, tray_length, tray_count=0):
        n = len(tray_poses)
        for i in range(n):
            tray_id = tray_count+i
            cpy= tray_d['sdf']['model']
            cpy['@name'] = u'tray_' + str(tray_id)
            posstr = str("%.3f %.3f 1.14965 0.0 0.0 0.0"%(tray_poses[i][0], tray_poses[i][1]))
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
            posstr = str("%.3f %.3f 1.19165 1.5707 0.0 0.0"%(trayp2_poses[i][0], trayp2_poses[i][1]))
            cpy['link']['pose']['#text'] = posstr
            geomstr = str("%.2f 0.085 0.005"%(tray_length))
            cpy['link']['collision']['geometry']['box']['size'] = geomstr
            cpy['link']['visual']['geometry']['box']['size'] = geomstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
            
    def add_pot21(self, pot21_d, pot21_poses, pot21_count=0):
        n = len(pot21_poses)
        for i in range(n):
            pot21_id = pot21_count+i
            cpy= pot21_d['sdf']['model']
            cpy['@name'] = u'pot21_' + str(pot21_id)
            posstr = str("%.3f %.3f 1.14965 1.5707 0.0 1.5707"%(pot21_poses[i][0], pot21_poses[i][1]))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
            
    def add_pot9(self, pot9_d, pot9_poses, pot9_count=0):
        n = len(pot9_poses)
        for i in range(n):
            pot9_id = pot9_count+i
            cpy= pot9_d['sdf']['model']
            cpy['@name'] = u'pot9_' + str(pot9_id)
            posstr = str("%.3f %.3f 1.14965 1.5707 0.0 1.5707"%(pot9_poses[i][0], pot9_poses[i][1]))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))   
            
            
    def add_pot1(self, pot1_d, pot1_poses, pot1_count=0):
        n = len(pot1_poses)
        for i in range(n):
            pot1_id = pot1_count+i
            cpy= pot1_d['sdf']['model']
            cpy['@name'] = u'pot1_' + str(pot1_id)
            posstr = str("%.3f %.3f 1.23465 0.0 0.0 0.0"%(pot1_poses[i][0], pot1_poses[i][1]))
            cpy['link']['pose']['#text'] = posstr
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
            
            
    def add_plant21(self, plant21_d, plant21_poses, plant21_count=0):
        n = len(plant21_poses)
        for i in range(n):
            plant21_id = plant21_count+i
            cpy= plant21_d['sdf']['model']
            cpy['@name'] = u'plant21_' + str(plant21_id)
            posstr = str("%.3f %.3f 1.06765 1.5707 0.0 1.5707"%(plant21_poses[i][0]-0.15, plant21_poses[i][1]-0.15))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))              
            

    def add_plant9(self, plant9_d, plant9_poses, plant9_count=0):
        n = len(plant9_poses)
        for i in range(n):
            plant9_id = plant9_count+i
            cpy= plant9_d['sdf']['model']
            cpy['@name'] = u'plant9_' + str(plant9_id)
            posstr = str("%.3f %.3f 1.24265 1.5707 0.0 1.5707"%(plant9_poses[i][0]-0.15, plant9_poses[i][1]-0.15))
            cpy['pose'] = posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))  

            
    def add_plant1(self, plant1_d, plant1_poses, plant1_count=0):
        n = len(plant1_poses)
        for i in range(n):
            plant1_id = plant1_count+i
            cpy= plant1_d['sdf']['model']
            cpy['@name'] = u'plant1_' + str(plant1_id)
            posstr = str("%.3f %.3f 1.23465 1.5707 0.0 0.0"%(plant1_poses[i][0], plant1_poses[i][1]))
            cpy['pose']=posstr
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
            
            
    def add_fixed_arches(self, fixed_arches_d, pose, j=0):
        cpy= fixed_arches_d['sdf']['model']
        cpy['@name'] = u'fixed_arches_' + str(j)
        posstr= str("%.3f %.3f 0.0 1.5707 0.0 1.5707"%(pose[0], pose[1]))
        cpy['pose'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))    
            
            
    def add_struts(self, strut_d, strut_poses, strut_roll, strut_count=0):
        n = len(strut_poses)
        for i in range(n):
            strut_id = strut_count+i
            cpy= strut_d['sdf']['model']
            cpy['@name'] = u'strut_' + str(strut_id)
            posstr= str("%.3f %.3f 0.536 %.3f 0.0 0.0"%(strut_poses[i][0], strut_poses[i][1], strut_roll))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
            
            
    def add_canopy10m(self, canopy10m_d, canopy10m_poses, canopy10m_count=0):
        n = len(canopy10m_poses)
        for i in range(n):
            canopy10m_id = canopy10m_count+i
            cpy= canopy10m_d['sdf']['model']
            cpy['@name'] = u'canopy10m_' + str(canopy10m_id)
            posstr= str("%.3f %.3f -0.00 1.5707 0.0 1.5707"%(canopy10m_poses[i][0], canopy10m_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))  
            

    def add_canopy4m(self, canopy4m_d, canopy4m_poses, canopy4m_count=0):
        n = len(canopy4m_poses)
        for i in range(n):
            canopy4m_id = canopy4m_count+i
            cpy= canopy4m_d['sdf']['model']
            cpy['@name'] = u'canopy4m_' + str(canopy4m_id)
            posstr= str("%.3f %.3f -0.00 1.5707 0.0 1.5707"%(canopy4m_poses[i][0], canopy4m_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))  
            
            
    def add_canopyhalfm(self, canopyhalfm_d, canopyhalfm_poses, canopyhalfm_count=0):
        n = len(canopyhalfm_poses)
        for i in range(n):
            canopyhalfm_id = canopyhalfm_count+i
            cpy= canopyhalfm_d['sdf']['model']
            cpy['@name'] = u'canopyhalfm_' + str(canopyhalfm_id)
            posstr= str("%.3f %.3f -0.00 1.5707 0.0 1.5707"%(canopyhalfm_poses[i][0], canopyhalfm_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
            

    def add_frontage(self, frontage_d, pose, j=0):
        cpy= frontage_d['sdf']['model']
        cpy['@name'] = u'frontage_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
        cpy['pose'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))              
            
            
    def add_riseholme_enclosure(self, riseholme_enclosure_d, pose, j=0):
        cpy= riseholme_enclosure_d['sdf']['model']
        cpy['@name'] = u'riseholme_enclosure_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
        
        
    def add_fhs(self, fhs_d, pose, j=0):
        cpy= fhs_d['sdf']['model']
        cpy['@name'] = u'food_handling_shed_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
        
        
    def add_fhs_floor(self, fhs_floor_d, pose, j=0):
        cpy= fhs_floor_d['sdf']['model']
        cpy['@name'] = u'fhs_floor_' + str(j)
        posstr= str("%.3f %.3f -0.001 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
        cpy['link']['pose']['#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
        

    def add_lab(self, lab_d, pose, j=0):
        cpy= lab_d['sdf']['model']
        cpy['@name'] = u'lab_' + str(j)
        posstr= str("%.3f %.3f 0.0 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))     
        
        
    def add_lab_floor(self, lab_floor_d, pose, j=0):
        cpy= lab_floor_d['sdf']['model']
        cpy['@name'] = u'lab_floor_' + str(j)
        posstr= str("%.3f %.3f -0.001 0.0 0.0 %.3f"%(pose[0], pose[1], pose[2]))
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