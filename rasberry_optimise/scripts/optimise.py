#!/usr/bin/env python
from __future__ import division
import rospy
from utils import *
from get_fitness import getFitness


if __name__ == "__main__":
    
    rospy.init_node("optimiser", anonymous=True, disable_signals=True)
    
    config_scenario = load_config_from_yaml("scenario.yaml")
    config_parameters = load_config_from_yaml("parameters.yaml")
    # Probably load a config file for the genetic algorithm here.

    # Initialise the genetic algorithm and fitness function here. 
    rcnfsrvs = config_parameters.keys()
    gf = getFitness(config_scenario, rcnfsrvs)  
    
    # Dummy genetic algorithm: just makes a dictionary with new (test) param values 
    # to pass to the fitness function.
    params = {}
    for i, rcnfsrv in enumerate(rcnfsrvs):
        params[rcnfsrv] = {}
        param_names = config_parameters.values()[i].keys()
        for param_name in param_names:
            if config_parameters.values()[i][param_name]['type'] == "float":
                params[rcnfsrv][param_name] = 1.1
            if config_parameters.values()[i][param_name]['type'] == "int":
                params[rcnfsrv][param_name] = 1
            if config_parameters.values()[i][param_name]['type'] == "bool":
                params[rcnfsrv][param_name] = "false"

    
    # Run the test scenario and get fitness.
    gf.run_scenario()
    gf.run_scenario(params)
#####################################################################################
