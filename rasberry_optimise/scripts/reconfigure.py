#!/usr/bin/env python
import rospy, sys, dynamic_reconfigure.client, rospkg
from rasberry_optimise.utils import *


def do_reconf(params):
    """Reconfigure parameters.
    """
    try:
        for param in params:
            print "Setting {} = {}".format("/".join((param["ns"], param["name"])), param["value"])
            rcnfclient = dynamic_reconfigure.client.Client(param["ns"], timeout=5.0)
            rcnfclient.update_configuration({param["name"]: param["value"]})
            
    except rospy.ROSException as e:
        rospy.logerr(e)
        rospy.signal_shutdown(e)
        exit()
#####################################################################################
        
        
#####################################################################################
if __name__ == "__main__":
    
    rospy.init_node("reconfigure", anonymous=True)
    
    if len(sys.argv) < 3:
        rospy.loginfo("usage is reconfigure.py parameters_yaml parameter_index")
        exit()
    else:
        print sys.argv
        print "\n"
        f_parameters = sys.argv[1]
        parameter_index = sys.argv[2]
        
        base_dir = rospkg.RosPack().get_path("rasberry_optimise") + "/resources/optim_params"
        parameter_sets = load_data_from_yaml(base_dir + "/" + f_parameters + ".yaml")
        
        parameters = parameter_sets[int(parameter_index)]
        
        do_reconf(parameters)
#####################################################################################