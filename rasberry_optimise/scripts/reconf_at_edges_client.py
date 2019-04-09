#!/usr/bin/env python


import rospy
from rasberry_optimise.srv import ReconfAtEdges

def reconf_at_edges_client(edge_id):
    rospy.wait_for_service('reconf_at_edges')
    try:
        reconf_at_edges = rospy.ServiceProxy('reconf_at_edges', ReconfAtEdges)
        resp1 = reconf_at_edges(edge_id)
        print resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    edge_id = "WayPoint46_WayPoint45"
    #edge_id = "WayPoint44_WayPoint43"

    reconf_at_edges_client(edge_id)
###############################################################################