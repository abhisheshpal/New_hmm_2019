#!/usr/bin/env python
from rasberry_optimise.utils import *
import rospy, rospkg, re, dynamic_reconfigure.client, sys
from copy import copy
from std_msgs.msg import String


class reconfAtEdges(object):
    
    def __init__(self, group_irn, group_utn, param_dict_irn, param_dict_utn):
        self.group_irn = group_irn
        self.group_utn = group_utn
        self.param_dict_irn = param_dict_irn
        self.param_dict_utn = param_dict_utn
        self.current_group = []
        self.regex = "(.*)--(.*)"
        
        self.rcnfclients_irn = self.make_reconf_clients(param_dict_irn)
        self.rcnfclients_utn = self.make_reconf_clients(param_dict_utn)
        
        rospy.Subscriber("/current_edge", String, self.current_edge_callback)
        
        
    def make_reconf_clients(self, param_dict):
        
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
                
        return rcnfclients
        
        
    def current_edge_callback(self, msg):
        
        if msg.data != "none":
            current_edge = re.match(self.regex, msg.data).groups()[0]
            
            if current_edge not in self.current_group:
                
                if current_edge in self.group_irn:
                    print "\n"
                    rospy.loginfo("RECONFIGURING MOVE-BASE FOR IN-ROW NAVIGATION")                    
                    
                    for rcnfsrv in self.param_dict_irn.keys():
                        self.do_reconf(self.rcnfclients_irn[rcnfsrv], self.param_dict_irn[rcnfsrv])
                    self.current_group = copy(self.group_irn)
                    
                if current_edge in self.group_utn:
                    print "\n"
                    rospy.loginfo("RECONFIGURING MOVE-BASE FOR U-TURN NAVIGATION")   
                    
                    for rcnfsrv in self.param_dict_utn.keys():
                        self.do_reconf(self.rcnfclients_utn[rcnfsrv], self.param_dict_utn[rcnfsrv])
                    self.current_group = copy(self.group_utn)   
            
            
    def do_reconf(self, rcnfclient, params):
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
            



if __name__ == "__main__":
    
    rospy.init_node("reconf_at_edges", anonymous=True, disable_signals=True)
    
    if len(sys.argv) < 4:
        rospy.loginfo("usage is reconf_at_edges.py omni")
        exit()
    else:
        print sys.argv
        print "\n"
        omni = sys.argv[1]
        parameter_index_irn = int(sys.argv[2])
        parameter_index_utn = int(sys.argv[3])
        
    rospack = rospkg.RosPack()
    base_dir = rospack.get_path("rasberry_optimise") + "/resources"  
    
    if omni == "true":
        rospy.loginfo("ASSUMING HOLONOMIC STEERING")
        print "\n"
        
        group_irn = load_data_from_json(base_dir + "/group_irn_omni.json")
        group_utn = load_data_from_json(base_dir + "/group_utn_omni.json")                    
        params_irn = load_data_from_json(base_dir + "/optim_params/dwa_irn_omni/dwa_irn_omni.json")[parameter_index_irn]
        params_utn = load_data_from_json(base_dir + "/optim_params/dwa_utn_omni/dwa_utn_omni.json")[parameter_index_utn]
        params_irn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_irn_omni/dwa_irn_omni.yaml")
        params_utn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_utn_omni/dwa_utn_omni.yaml")
               
    elif omni == "false":
        rospy.loginfo("ASSUMING DIFFERENTIAL DRIVE STEERING")
        print "\n"
        
        group_irn = load_data_from_json(base_dir + "/group_irn.json")
        group_utn = load_data_from_json(base_dir + "/group_utn.json") 
        params_irn = load_data_from_json(base_dir + "/optim_params/dwa_irn/dwa_irn.json")[parameter_index_irn]
        params_utn = load_data_from_json(base_dir + "/optim_params/dwa_utn/dwa_utn.json")[parameter_index_utn]
        params_irn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_irn/dwa_irn.yaml")
        params_utn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_utn/dwa_utn.yaml")
        
    else:
        rospy.logerr("Please set omni = \"true\" or omni = \"false\"")
        exit()
        
    param_dict_irn = make_param_dict(params_irn_config, params_irn)
    param_dict_utn = make_param_dict(params_utn_config, params_utn)
    
    reconfAtEdges(group_irn, group_utn, param_dict_irn, param_dict_utn)
        
    rospy.spin()
#####################################################################################