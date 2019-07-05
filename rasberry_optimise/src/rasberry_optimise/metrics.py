#!/usr/bin/env python
from __future__ import division
import numpy as np


def scorepath(path, tickspersec=10, fun_radspersec2dollarspersec=(lambda dth : -dth ** 2)):
    """Smoothness score of the robot's trajectory.
    """
    #convert utility to function from seconds to ticks
    fun_radspertick2dollarspertick = (lambda secspertick : (lambda fun_radpersec2dollarspersec : (lambda dth : fun_radspersec2dollarspersec(dth*tickspersec) / tickspersec )))(tickspersec)(fun_radspersec2dollarspersec)
    
    #rotations per tick in the robot path
    dths = np.abs(np.diff(path[:,2]))

    L = list(map(fun_radspertick2dollarspertick, dths))   #lagrangian

    rotation_cost = -np.sum(L)
    return rotation_cost
    

def get_trajectory_length(trajectory):
    """Calculate the length of the robot's trajectory.
    """
    trajectory = np.array(trajectory)[:, :2]
    dx = np.diff(trajectory[:, 0])
    dy = np.diff(trajectory[:, 1])
    d = np.sqrt(dx**2 + dy**2)
    
    return np.sum(d)
    

def get_path_error(trajectory, path):
    
    eds = []
    trajectory_at_path = get_trajectory_at_path(trajectory, path)
    for pos, path_pos in zip(trajectory_at_path, path):
        dx = pos[0] - path_pos[0]
        dy = pos[1] - path_pos[1]
        eds.append(np.sqrt(dx**2 + dy**2))
    return np.mean(eds)
    
    
def get_trajectory_at_path(trajectory, path):    
    
    trajectory_at_path = []    
    trajectory = np.array(trajectory)[:, :2]
    for pos in path:
        dx = trajectory[:, 0] - pos[0]
        dy = trajectory[:, 1] - pos[1]
        ed = np.sqrt(dx**2 + dy**2)
        index = np.argmin(ed)
        pos_at_path = [trajectory[index][0], trajectory[index][1]]
        trajectory_at_path.append(pos_at_path)
    return trajectory_at_path
    
    
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
    
    position_error = np.mean(np.sqrt(dxs**2 + dys**2))
    orientation_error = np.mean(dthetas) / (np.pi/180)     
    
    return position_error, orientation_error
#####################################################################################