#!/usr/bin/env python
import rospy, sys, dynamic_reconfigure.client
from rasberry_optimise.utils import *


def do_reconf(rcnfclient, params):
    """Reconfigure parameters.
    """
    try:
        for param in params.keys():
            print "Setting {} = {}".format(param, params[param]) 
        rcnfclient.update_configuration(params)
    except rospy.ROSException as e:
        rospy.logerr(e)
        rospy.signal_shutdown(e)
        exit()
#####################################################################################
        
        
#####################################################################################
if __name__ == "__main__":
    
    rospy.init_node("reconfigure", anonymous=True)
    
    if len(sys.argv) < 3:
        rospy.loginfo("usage is reconfigure.py path_to_parameters_json parameter_index path_to_parameters_yaml")
        exit()
    else:
        print sys.argv
        print "\n"
        path_to_parameters = sys.argv[1]
        parameter_index = sys.argv[2]
        path_to_config_params = sys.argv[3]
        
        parameter_sets = load_data_from_json(path_to_parameters)
        config_params = load_data_from_yaml(path_to_config_params)        
        
        parameters = parameter_sets[int(parameter_index)]
        param_dict = make_param_dict(config_params, parameters)
        
        # Create reconfigure clients.
        rospy.loginfo("Creating reconfigure clients ...")
        rcnfsrvs = param_dict.keys()
        rcnfclients = {}
        for rcnfsrv in rcnfsrvs:
            try:
                rcnfclients[rcnfsrv] \
                = dynamic_reconfigure.client.Client(rcnfsrv, timeout=5.0)
                rospy.loginfo("Created client for {}".format(rcnfsrv))
            except rospy.ROSException as e:
                rospy.logerr(e)
                rospy.signal_shutdown(e)
                exit()
        
        print "\n"        
        for rcnfsrv in rcnfsrvs:
            do_reconf(rcnfclients[rcnfsrv], param_dict[rcnfsrv])
#####################################################################################
