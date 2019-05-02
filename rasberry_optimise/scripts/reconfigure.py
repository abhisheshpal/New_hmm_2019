#!/usr/bin/env python
import rospy, sys, dynamic_reconfigure.client, rospkg
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
        rospy.loginfo("usage is reconfigure.py parameter_dir parameter_index")
        exit()
    else:
        print sys.argv
        print "\n"
        parameter_dir = sys.argv[1]
        parameter_index = sys.argv[2]
        
        rospack = rospkg.RosPack()
        base_dir = rospack.get_path("rasberry_optimise") + "/resources/optim_params"
        
        parameter_sets = load_data_from_json(base_dir + "/" + parameter_dir + "/" + parameter_dir + ".json")
        config_params = load_data_from_yaml(base_dir + "/" + parameter_dir + "/" + parameter_dir + ".yaml")        
        
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