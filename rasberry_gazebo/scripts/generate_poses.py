# -*- coding: utf-8 -*-
"""
Created on Sun Mar 25 15:20:20 2018

@author: Adam Binch
@email: adambinch@gmail.com
"""
################################################################################
from __future__ import division
import numpy as np
from itertools import product
from utils import to_list



def get_pole_poses(nx, ny, dx, dy, xoffset, yoffset):
    xposes = np.arange(0, nx*dx, dx) + xoffset
    yposes = np.arange(0, ny*dy, dy) + yoffset
    poses = []
    for i, j in list(product(range(nx), range(ny))):
        pose = [xposes[i], yposes[j]]
        poses.append(pose)
    return poses, xposes, yposes
  
  
    
def get_tray_poses(nx, ny, dx, xoffset, yposes):
    tray_length = ((nx-1) * dx) + 0.1
    tray_xpose = tray_length/2 + xoffset - 0.05
    tray_xposes = np.zeros(ny) + tray_xpose
    return to_list(tray_xposes, yposes), tray_length



def get_trayp2_poses(tray_length, pole_xoffset, pole_ny, pole_yposes):
    trayp2_xpose = tray_length/2 + pole_xoffset - 0.05
    trayp2_xposes = np.zeros(pole_ny*2) + trayp2_xpose
    a = np.empty(pole_ny)
    b = np.empty(pole_ny)
    for i in range(pole_ny):
        a[i] = pole_yposes[i] - 0.075
        b[i] = pole_yposes[i] + 0.075
    trayp2_yposes = np.hstack((a, b))            
    return to_list(trayp2_xposes, trayp2_yposes)
    
    
    
def get_pot21_poses(pole_xposes, pole_yposes):      
    pot21_length = 10
    row_length = pole_xposes[-1] - pole_xposes[0]
    remainder = np.mod(row_length, pot21_length)
    pot21_max_length = row_length - remainder
    
    xposes = np.arange(pole_xposes[0], pole_xposes[0]+pot21_max_length, pot21_length)
    poses = []
    for i, j in list(product(range(len(xposes)), range(len(pole_yposes)))):
        pose = [xposes[i]-0.07, pole_yposes[j]-0.07]
        poses.append(pose)
    return poses, pot21_max_length    


    
def get_pot9_poses(pole_xposes, pole_yposes, pot21_max_length):      
    pot9_length = 4
    row_length = (pole_xposes[-1] - pole_xposes[0]) - pot21_max_length
    remainder = np.mod(row_length, pot9_length)
    pot9_max_length = row_length - remainder
    
    xposes = np.arange(pole_xposes[0]+pot21_max_length, pole_xposes[0]+pot21_max_length+pot9_max_length, pot9_length)
    poses = []
    for i, j in list(product(range(len(xposes)), range(len(pole_yposes)))):
        pose = [xposes[i]-0.07, pole_yposes[j]-0.07]
        poses.append(pose)
    return poses, pot9_max_length
    

    
def get_pot1_poses(pole_xposes, pole_yposes, pot21_max_length, pot9_max_length):
    pot1_dx = 0.5
    pot1_xposes = np.arange(pole_xposes[0]+pot21_max_length+pot9_max_length, pole_xposes[1], pot1_dx)
    pot1_poses = []
    for i, j in list(product(range(len(pot1_xposes)), range(len(pole_yposes)))):
        pot1_pose = [pot1_xposes[i], pole_yposes[j]]
        pot1_poses.append(pot1_pose)
    return pot1_poses, pot1_xposes
    
    
    
def get_plant_poses(pot1_xposes, pot1_yposes):
    plant1_poses = []
    for i, j in list(product(range(len(pot1_xposes)), range(len(pot1_yposes)))):
        plant1_pose = [pot1_xposes[i]-0.005, pot1_yposes[j]-0.38, 1.282, 0.785]
        plant1_poses.append(plant1_pose)
    
    plant2_poses = []
    for i, j in list(product(range(len(pot1_xposes)), range(len(pot1_yposes)))):
        plant2_pose = [pot1_xposes[i]-0.2725, pot1_yposes[j]-0.2645, 1.408, 0.000]
        plant2_poses.append(plant2_pose)    
    return plant1_poses + plant2_poses
    
    
    
def get_arch_poses(arch_nx, arch_dx, arch_xoffset, pole_dy, pole_yposes):
    arch_xposes = np.arange(0, arch_nx*arch_dx, arch_dx) + arch_xoffset
    arch_yposes = np.zeros(arch_nx) + pole_yposes[0] - pole_dy
    return to_list(arch_xposes, arch_yposes), arch_xposes, arch_yposes
    
    
    
def get_canopy10m_poses(arch_xposes, arch_ypose, pole_dy):
    canopy10m_length = 10
    arch_length = arch_xposes[1] - arch_xposes[0]
    remainder = np.mod(arch_length, canopy10m_length)
    canopy10m_max_length = arch_length - remainder    
    
    canopy10m_xposes = np.arange(arch_xposes[0], arch_xposes[0]+canopy10m_max_length, canopy10m_length)
    canopy10m_yposes = np.zeros(len(canopy10m_xposes)) + arch_ypose + (3*pole_dy) + 0.017
    return to_list(canopy10m_xposes, canopy10m_yposes), canopy10m_max_length    
    
    

def get_canopy4m_poses(arch_xposes, arch_ypose, pole_dy, canopy10m_max_length):
    canopy4m_length = 4
    arch_length = (arch_xposes[1] - arch_xposes[0]) - canopy10m_max_length
    remainder = np.mod(arch_length, canopy4m_length)
    canopy4m_max_length = arch_length - remainder    
    
    canopy4m_xposes = np.arange(arch_xposes[0]+canopy10m_max_length, arch_xposes[0]+canopy10m_max_length+canopy4m_length, canopy4m_length)
    canopy4m_yposes = np.zeros(len(canopy4m_xposes)) + arch_ypose + (3*pole_dy) + 0.017
    return to_list(canopy4m_xposes, canopy4m_yposes), canopy4m_max_length
    
    
    
def get_canopyhalfm_poses(arch_xposes, arch_ypose, pole_dy, canopy10m_max_length, canopy4m_max_length):
    canopyhalfm_length = 0.5    
    canopyhalfm_xposes = np.arange(arch_xposes[0]+canopy10m_max_length+canopy4m_max_length, arch_xposes[1], canopyhalfm_length)
    canopyhalfm_yposes = np.zeros(len(canopyhalfm_xposes)) + arch_ypose + (3*pole_dy) + 0.017
    return to_list(canopyhalfm_xposes, canopyhalfm_yposes)   
################################################################################