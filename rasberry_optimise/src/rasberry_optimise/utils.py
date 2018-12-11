#!/usr/bin/env python
import yaml

def load_config_from_yaml(config_filename):
    with open(config_filename,'r') as f:
        return yaml.load(f)
#####################################################################################