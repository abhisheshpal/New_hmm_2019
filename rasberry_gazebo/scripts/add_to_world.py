# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 13:55:50 2018

@author: adam
"""
################################################################################
import copy


class AddtoWorld(object):
    
    def __init__(self, world_d):
        
        world_d['sdf']['world']['model']=[] # creates a list to store the models
        self.world_d = world_d
        
    def add_poles(self, pole_d, pole_poses, j=0):
        n = len(pole_poses)
        for i in range(n):
            pole_id = (j*n)+i
            cpy= pole_d['sdf']['model']
            cpy['@name'] = u'unit_cylinder_' + str(pole_id)
            posstr= str("%.2f %.2f 0.691175 0.0 0.0 0.0"%(pole_poses[i][0], pole_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))      

    def add_trays(self, tray_d, tray_poses, tray_length, j=0):
        n = len(tray_poses)
        for i in range(n):
            tray_id = (j*n)+i
            cpy= tray_d['sdf']['model']
            cpy['@name'] = u'tray_' + str(tray_id)
            posstr = str("%.2f %.2f 1.38235 0.0 0.0 0.0"%(tray_poses[i][0], tray_poses[i][1]))
            cpy['link']['collision']['pose']['#text'] = posstr
            cpy['link']['visual']['pose']['#text'] = posstr
            geomstr = str("%.2f 0.15 0.02"%(tray_length))
            cpy['link']['collision']['geometry']['box']['size'] = geomstr
            cpy['link']['visual']['geometry']['box']['size'] = geomstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))

    def add_arches(self, arch_d, arch_poses, j=0):
        n = len(arch_poses)
        for i in range(n):
            arch_id = (j*n)+i
            cpy= arch_d['sdf']['model']
            cpy['@name'] = u'dummy_arch_' + str(arch_id)
            posstr= str("%.2f %.2f 0.0 1.570796 0.0 1.570796"%(arch_poses[i][0], arch_poses[i][1]))
            cpy['pose']=posstr
            self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))
            
    def add_riseholme_enclosure(self, riseholme_enclosure_d, pose, j=0):
        cpy= riseholme_enclosure_d['sdf']['model']
        cpy['@name'] = u'riseholme_enclosure_' + str(j)
        posstr= str("%.2f %.2f 0.0 0.0 0.0 0.0"%(pose[0], pose[1]))
        cpy[u'pose'][u'#text'] = posstr
        self.world_d['sdf']['world']['model'].append(copy.deepcopy(cpy))           
################################################################################            