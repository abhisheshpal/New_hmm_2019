#!/usr/bin/env python
from __future__ import division
import numpy as np
from pylab import *


def scorepath(path, tickspersec=10, fun_radspersec2dollarspersec=(lambda dth : -dth ** 2)):
    """Smoothness score of the robot's trajectory.
    """
    #convert utility to function from seconds to ticks
    fun_radspertick2dollarspertick = (lambda secspertick : (lambda fun_radpersec2dollarspersec : (lambda dth : fun_radspersec2dollarspersec(dth*tickspersec) / tickspersec )))(tickspersec)(fun_radspersec2dollarspersec)
    
    #rotations per tick in the robot path
    dths = abs(diff(path[:,2]))

    L = list(map(fun_radspertick2dollarspertick, dths))   #lagrangian

    cost_dollars = sum(L)
    return cost_dollars
    

def get_trajectory_length(trajectory):
    """Calculate the length of the robot's trajectory.
    """
    trajectory = np.array(trajectory)[:, :2]
    dx = np.diff(trajectory[:, 0])
    dy = np.diff(trajectory[:, 1])
    d = np.sqrt(dx**2 + dy**2)
    
    return np.sum(d)
    

def get_dist_from_coords(coords, trajectory):
    """
    """
    trajectory = np.array(trajectory)[:, :2]
    coords = np.array(coords)
    indices_right = np.where(coords[:, 2] == 0)[0]
    indices_left = np.where(coords[:, 2] == 1)[0]
    coords_right = coords[:, :2][indices_right]
    coords_left = coords[:, :2][indices_left]
    
    ses = np.empty(trajectory.shape[0])
    for i in range(trajectory.shape[0]):
        pos = trajectory[i, :]
        
        dx_right = coords_right[:, 0] - pos[0]
        dy_right = coords_right[:, 1] - pos[1]
        d_right = np.sqrt(dx_right**2 + dy_right**2)
        closest_right = np.sort(d_right)[:2]
        
        dx_left = coords_left[:, 0] - pos[0]
        dy_left = coords_left[:, 1] - pos[1]
        d_left = np.sqrt(dx_left**2 + dy_left**2)
        closest_left = np.sort(d_left)[:2]
        
        closest = np.hstack((closest_left, closest_right))
        ses[i] = np.sum(closest**2)
        
    return np.mean(ses)
    
    
def get_localisation_error(trajectory_ground_truth, trajectory_amcl):
    """
    """
    
    trajectory_ground_truth = np.array(trajectory_ground_truth)
    trajectory_amcl = np.array(trajectory_amcl)
    
    xs_ground_truth = trajectory_ground_truth[:, 0]
    ys_ground_truth = trajectory_ground_truth[:, 1]  
    thetas_ground_truth = trajectory_ground_truth[:, 2] 
    
    xs_amcl = trajectory_amcl[:, 0]
    ys_amcl = trajectory_amcl[:, 1] 
    thetas_amcl = trajectory_amcl[:, 2]
    
    dxs = xs_ground_truth - xs_amcl
    dys = ys_ground_truth - ys_amcl
    dthetas = 180 - np.abs(np.abs(thetas_ground_truth - thetas_amcl) - 180)
    
    euclids = np.sqrt(dxs**2 + dys**2 + dthetas**2)
    euclids_positions = np.sqrt(dxs**2 + dys**2)
    
    pose_error = np.mean(euclids)
    position_error = np.mean(euclids_positions)
    orientation_error = np.mean(dthetas) / (np.pi/180)     
    
    return pose_error, position_error, orientation_error
#####################################################################################