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
    traj = np.array(trajectory)[:, :2]
    dx = np.diff(traj[:, 0])
    dy = np.diff(traj[:, 1])
    d = np.sqrt(dx**2 + dy**2)
    
    return np.sum(d)
    

def get_dist_from_coords(coords, trajectory):
    """Calculate average squared distance from coordinates to each point on the 
       robot's trajectory.
    """
    traj = np.array(trajectory)[:, :2]
    d = np.empty((len(coords), traj.shape[0]))
    for i, coord in enumerate(coords):
        dx = coord[0] - traj[:, 0]
        dy = coord[1] - traj[:, 1]
        #d[i, :] = np.sqrt(dx**2 + dy**2)
        d[i, :] = dy**2
        
    return (1.0 / traj.shape[0]) * np.sum(d) #np.sum(d**2)
#####################################################################################