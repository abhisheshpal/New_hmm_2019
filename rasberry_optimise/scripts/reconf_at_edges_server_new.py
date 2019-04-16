#!/usr/bin/env python
import rospy
from strands_navigation_msgs.srv import ReconfAtEdges, ReconfAtEdgesResponse
from dynamic_reconfigure.srv import ReconfigureRequest, Reconfigure


class reconf_at_edges_server(object):
    
    def __init__(self):    
    
        self.data = rospy.get_param("/reconf_at_edges_new")
        
        rospy.Service('reconf_at_edges', ReconfAtEdges, self.handle_reconf_at_edges)
        
        rospy.wait_for_service('/move_base/DWAPlannerROS/set_parameters', timeout=4)
        try:
            self.service_client = rospy.ServiceProxy("/move_base/DWAPlannerROS/set_parameters", Reconfigure)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
            
    def handle_reconf_at_edges(self, req):
        pass


if __name__ == "__main__":

    rospy.init_node('reconf_at_edges_server', anonymous=True)
    reconf_at_edges_server()    
    
    
    
    
    