#!/usr/bin/env python
import rospy, re
from strands_navigation_msgs.srv import ReconfAtEdges, ReconfAtEdgesResponse
from dynamic_reconfigure.srv import ReconfigureRequest, Reconfigure
from dynamic_reconfigure.msg import DoubleParameter, IntParameter, BoolParameter


class set_params_at_edges_server(object):
    
    
    def __init__(self):    
    
        self.data = rospy.get_param("/set_params_at_edges_server")
        
        rospy.Service('set_params_at_edges', ReconfAtEdges, self.handle_set_params_at_edges)
        
        rospy.wait_for_service('/move_base/DWAPlannerROS/set_parameters', timeout=4.0)
        try:
            self.service_client = rospy.ServiceProxy("/move_base/DWAPlannerROS/set_parameters", Reconfigure)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        self.regex = "(.*)/(.*)/(.*)"
            
            
    def handle_set_params_at_edges(self, req):
        edge_id = req.edge_id        
        request = ReconfigureRequest()
        
        for key in self.data.keys():
            if edge_id in self.data[key]["edges"]:
                print "\n"
                rospy.loginfo("Setting move-base parameters for edge group {} ...".format(key))
                
                for param in self.data[key]["parameters"]:
                    
                    print "Setting {} = {}".format(param["name"], param["value"])
                    
                    if param["type"] == "double":
                        p = DoubleParameter()
                    elif param["type"] == "int":
                        p = IntParameter()
                    elif param["type"] == "bool":
                        p = BoolParameter()
                    else:
                        rospy.logerr("Parameter type {} not supported".format(param["type"]))
                        
                    p.name = re.match(self.regex, param["name"]).groups()[-1]
                    p.value = param["value"]
                    
                    if param["type"] == "double":
                        request.config.doubles.append(p)
                    elif param["type"] == "int":
                        request.config.ints.append(p)
                    elif param["type"] == "bool":
                        request.config.bools.append(p)
                
                try:
                    self.service_client(request)
                    success = True
                except rospy.ROSException:
                    success = False
                
        return ReconfAtEdgesResponse(success)
                        
                        


if __name__ == "__main__":

    rospy.init_node('set_params_at_edges_server', anonymous=True)
    reconf_at_edges_server()    
    rospy.spin()
#####################################################################################
