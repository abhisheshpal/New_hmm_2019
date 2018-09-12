# -*- coding: utf-8 -*-
"""
Created on Sun Mar 25 15:21:56 2018

@author: Adam Binch
@email: adambinch@gmail.com
"""
################################################################################
import numpy as np
import json, xmltodict, yaml


def load_data_from_yaml(filename):
    with open(filename, 'r') as f:
        return yaml.load(f)


def load_data_from_xml(filename):
    with open(filename) as f:
        doc = json.dumps(xmltodict.parse(f.read()))
        return json.loads(doc)


def to_list(array1, array2):        
    return np.ndarray.tolist(np.vstack((array1, array2)).T) 
################################################################################