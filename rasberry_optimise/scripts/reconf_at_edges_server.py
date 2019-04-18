#!/usr/bin/env python
import rospy, dynamic_reconfigure.client
from strands_navigation_msgs.srv import ReconfAtEdges, ReconfAtEdgesResponse


class reconf_at_edges_server(object):
    

    def __init__(self):    
    
        self.config = rospy.get_param("/reconf_at_edges_server")
        rospy.Service('reconf_at_edges', ReconfAtEdges, self.handle_reconf_at_edges)


    def handle_reconf_at_edges(self, req):
        
        success = False
        for key in self.config.keys():
            
            if req.edge_id in self.config[key]["edges"]:
                print "\n"
                rospy.loginfo("Setting parameters for edge group {} ...".format(key))
                
                try: 
                    for param in self.config[key]["parameters"]:
                        
                        rcnfclient = dynamic_reconfigure.client.Client(param["ns"], timeout=5.0)
                        rcnfclient.update_configuration({param["name"]: param["value"]})                        
                        
                        print "Setting {} = {}".format(param["ns"] + "/" + param["name"], param["value"])
                    
                    success = True
                        
                except rospy.ROSException as e:
                    rospy.logerr(e)

                
        return ReconfAtEdgesResponse(success)
                    
                 
                 
                 
if __name__ == "__main__":

    rospy.init_node('reconf_at_edges_server', anonymous=True)
    reconf_at_edges_server()
    rospy.spin()
#####################################################################################