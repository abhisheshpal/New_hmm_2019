#!/usr/bin/env python
from roslib.packages import find_resource
import yaml

def load_config_from_yaml(config_filename):
    config_file = find_resource("rasberry_optimise", config_filename)[0]
    with open(config_file,'r') as f:
        return yaml.load(f)
#####################################################################################        