# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 13:42:25 2018

@author: Adam Binch

Calculate new node postions on the topological map (for the simulation) that are 
centred between the poles and write them to a text file.
"""
#####################################################################################
import numpy as np
import matplotlib.pyplot as plt
from itertools import product


def get_pole_positions(nx, ny, dx, dy, xoffset, yoffset):
    xposes = np.arange(0, nx*dx, dx) + xoffset
    yposes = np.arange(0, ny*dy, dy) + yoffset
    poses = []
    for i, j in list(product(range(nx), range(ny))):
        pose = [xposes[i], yposes[j]]
        poses.append(pose)
    return np.array(poses)
    
    
def get_centres(nx, ny, dx, dy, xoffset, yoffset):
    xposes = np.arange(-0.5*dx, nx*dx+(0.5*dx)+dx, dx) + xoffset
    yposes = np.arange(-0.5*dy, ny*dy+(0.5*dy)+dy, dy) + yoffset
    centres = []
    for i, j in list(product(range(ny+1), range(nx+1))):
        pose = [xposes[j], yposes[i]]
        centres.append(pose)
    return np.array(centres)    
#####################################################################################
    
    
#####################################################################################
plot = True    
write_to_file = False
    
pole_nx = 9
pole_ny = 5
pole_dx = 2.94
pole_dy = 1.32
pole_xoffset = 7

pole_yoffset_pTunnelA = -6.64
pole_yoffset_pTunnelB = 1.32

yoffset_c1 = -5*pole_dy
yoffset_c2 =  1.32

posA = \
get_pole_positions(pole_nx, pole_ny, pole_dx, pole_dy, pole_xoffset, pole_yoffset_pTunnelA) 

posB = \
get_pole_positions(pole_nx, pole_ny, pole_dx, pole_dy, pole_xoffset, pole_yoffset_pTunnelB) 

cA = \
get_centres(pole_nx, pole_ny, pole_dx, pole_dy, pole_xoffset, yoffset_c1) 

cB = \
get_centres(pole_nx, pole_ny, pole_dx, pole_dy, pole_xoffset, yoffset_c2) 

# way point numbers for polytunnel A
way_points = [65,6,2,1,5,4,3,7,8,9]
way_points.extend([64,18,17,16,15,14,13,12,11,10])
way_points.extend([60,19,20,21,22,23,24,25,26,27])
way_points.extend([59,28,29,30,31,32,33,34,35,36])
way_points.extend([58,37,38,39,40,41,42,43,44,45])
way_points.extend([57,54,53,52,51,50,49,48,47,46])
way_points = np.array(way_points).reshape(len(way_points), 1)

cA = np.hstack((way_points, cA))
cB = np.hstack((way_points + 74, cB))

if plot:
    plt.figure(1); plt.clf()
    plt.plot(posA[:, 0], posA[:, 1], 'b.')
    plt.plot(cA[:, 1], cA[:, 2], 'r.')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Polytunnel 1')
    plt.legend(['Poles', 'Centres'], frameon=False)
    plt.axis('equal')
    
    plt.figure(2); plt.clf()
    plt.plot(posB[:, 0], posB[:, 1], 'b.')
    plt.plot(cB[:, 1], cB[:, 2], 'r.')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Polytunnel 2')
    plt.legend(['Poles', 'Centres'], frameon=False)
    plt.axis('equal')
    
if write_to_file:
    np.savetxt('centres_polytunnelA_sim.txt', cA, fmt='%1.3f')
    np.savetxt('centres_polytunnelB_sim.txt', cB, fmt='%1.3f')    
#####################################################################################