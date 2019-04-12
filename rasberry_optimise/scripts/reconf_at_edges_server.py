#!/usr/bin/env python

import rospy, rospkg, dynamic_reconfigure.client, sys
from strands_navigation_msgs.srv import ReconfAtEdges, ReconfAtEdgesResponse
from rasberry_optimise.utils import *


class reconf_at_edges_server(object):

    def __init__(self, omni):

        rospack = rospkg.RosPack()
        base_dir = rospack.get_path("rasberry_optimise") + "/resources"

        if omni == "true":
            rospy.loginfo("ASSUMING HOLONOMIC STEERING")
            print "\n"

            self.group_irn = load_data_from_json(base_dir + "/group_irn_omni.json")
            self.group_utn = load_data_from_json(base_dir + "/group_utn_omni.json")
            params_irn = load_data_from_json(base_dir + "/optim_params/dwa_irn_omni/dwa_irn_omni.json")[0]
            params_utn = load_data_from_json(base_dir + "/optim_params/dwa_utn_omni/dwa_utn_omni.json")[0]
            params_irn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_irn_omni/dwa_irn_omni.yaml")
            params_utn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_utn_omni/dwa_utn_omni.yaml")

        else:
            rospy.loginfo("ASSUMING DIFFERENTIAL DRIVE STEERING")
            print "\n"

            self.group_irn = load_data_from_json(base_dir + "/group_irn.json")
            self.group_utn = load_data_from_json(base_dir + "/group_utn.json")
            params_irn = load_data_from_json(base_dir + "/optim_params/dwa_irn/dwa_irn.json")[0]
            params_utn = load_data_from_json(base_dir + "/optim_params/dwa_utn/dwa_utn.json")[1]
            params_irn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_irn/dwa_irn.yaml")
            params_utn_config = load_data_from_yaml(base_dir + "/optim_params/dwa_utn/dwa_utn.yaml")

        self.param_dict_irn = make_param_dict(params_irn_config, params_irn)
        self.param_dict_utn = make_param_dict(params_utn_config, params_utn)

        self.rcnfclients_irn = self.make_reconf_clients(self.param_dict_irn)
        self.rcnfclients_utn = self.make_reconf_clients(self.param_dict_utn)

        rospy.Service('reconf_at_edges', ReconfAtEdges, self.handle_reconf_at_edges)


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


    def handle_reconf_at_edges(self, req):
        edge_id = req.edge_id

        if edge_id in self.group_irn:
            print "\n"
            rospy.loginfo("RECONFIGURING MOVE-BASE FOR IN-ROW NAVIGATION")

            for rcnfsrv in self.param_dict_irn.keys():
                self.do_reconf(self.rcnfclients_irn[rcnfsrv], self.param_dict_irn[rcnfsrv])
            success = True

        elif edge_id in self.group_utn:
            print "\n"
            rospy.loginfo("RECONFIGURING MOVE-BASE FOR U-TURN NAVIGATION")

            for rcnfsrv in self.param_dict_utn.keys():
                self.do_reconf(self.rcnfclients_utn[rcnfsrv], self.param_dict_utn[rcnfsrv])
            success = True

        else:
            rospy.loginfo("Requested edge_id not in any specified groups")
            success = False

        return ReconfAtEdgesResponse(success)


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

    if len(sys.argv) < 2:
        rospy.loginfo("usage is reconf_at_edges_server.py omni")
        exit()
    else:
        print sys.argv
        print "\n"
        omni = sys.argv[1]

    rospy.init_node('reconf_at_edges_server')
    reconf_at_edges_server(omni)
    rospy.spin()
#####################################################################################
