#!/usr/bin/env python
import rospy, sys
from rasberry_optimise.utils import *
from rasberry_optimise.rasberry_scenario_server import scenario_server


if __name__ == "__main__":

    rospy.init_node("optimiser", anonymous=True, disable_signals=True)

    if len(sys.argv) < 3:
        rospy.loginfo("usage is optimise.py path_to_scenario_yaml path_to_parameters_yaml")
        exit()
    else:
        print sys.argv
        scenario = sys.argv[1]
        parameters = sys.argv[2]

    config_scenario = load_config_from_yaml(scenario)
    config_parameters = load_config_from_yaml(parameters)
    # Probably load a config file for the genetic algorithm here.

    # Initialise the genetic algorithm and scenario server (fitness function) here.
    rcnfsrvs = config_parameters.keys()
    ss = scenario_server(config_scenario, rcnfsrvs)

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
    ss.run_scenario()
    ss.run_scenario(params)
#####################################################################################