# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 12:53:17 2018

@author: adam
"""
#####################################################################################
import numpy as np, yaml


def get_positions(filename):
    
    with open(filename) as f:
        content = f.readlines()
        content = [x.strip() for x in content] 
        
    vals=[]
    for i in content:
        d=i.split()
        vals.append(d)

    for i in vals:
        i[0]=float(i[0])
        i[1]=float(i[1])
        i[2]=float(i[2])
        
    return np.array(vals)
    

def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.load(f)    
#####################################################################################    