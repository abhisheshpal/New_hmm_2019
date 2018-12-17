#!/usr/bin/env python
from __future__ import division
import rospy, actionlib, dynamic_reconfigure.client
from gazebo_msgs.srv import SetModelState 
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from topological_navigation import route_search
from strands_navigation_msgs.msg import TopologicalMap
from std_srvs.srv import Empty
from utils import *


class scenario_server(object):
    """getFitness class definition.    
    """
    
    def __init__(self, config_scenario, rcnfsrvs=None):
        """
        Keyword arguments:
        config -- dictionary containing the configuration parameters for running a 
                  test scenario.
        """
        # Get config for test scenario.
        self.start_node = config_scenario["start_node"]
        self.goal_node = config_scenario["goal_node"]
        self.robot_name = config_scenario["robot_name"]
        self.max_wait_time = rospy.Duration(config_scenario["max_wait_time"])
        self.rcnfsrvs = rcnfsrvs

        
        try:
            assert self.start_node != self.goal_node
        except AssertionError:
            rospy.logerr("Start node should not be the same as the goal node.")
            rospy.signal_shutdown("Start node should not be the same as the goal node.")
            exit()
            
        
        # Get the topological map.
        self.topo_map = None
        self.rec_map = False
        rospy.Subscriber("/topological_map", TopologicalMap, self.map_callback)
        rospy.loginfo("Waiting for Topological map ...")
        while not self.rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        rospy.loginfo("Topological map received.")
        
        
        # Check that start/goal nodes and route between them exist.
        self.lookup_node(self.start_node)
        self.lookup_node(self.goal_node)
        rospy.loginfo("Looking up route between {} and {} ..."
                      .format(self.start_node, self.goal_node))
        self.router = route_search.TopologicalRouteSearch(self.topo_map)
        self.lookup_route(self.start_node, self.goal_node)
        
        
        # For teleporting the robot model in gazebo.
        try:
            self.set_model_state_client = \
            rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
        except rospy.ROSException as e:
            rospy.logerr(e)
            rospy.signal_shutdown(e)
            exit()
            
        
        try:
            self.clear_costmaps_client = \
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=5.0)
        except rospy.ROSException as e:
            rospy.logerr(e)
            rospy.signal_shutdown(e)
            exit()
            
            
        # Topological navigation setup.
        self.topo_nav_client = \
        actionlib.SimpleActionClient("/topological_navigation", GotoNodeAction)
        rospy.loginfo("Waiting for action server ...")
        wait = self.topo_nav_client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available.")
            rospy.signal_shutdown("Action server not available.")
            exit()
        rospy.loginfo("Connected to action server.")
        self.topo_goal = GotoNodeGoal(target=self.goal_node)
        
        
        # For resetting the pose estimate of the robot model.
        self.init_pose_pub = \
        rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        
        
        # Create reconfigure clients.
        if self.rcnfsrvs is not None:
            rospy.loginfo("Creating reconfigure clients ...")
            self.rcnfclients = {}
            for rcnfsrv in self.rcnfsrvs:
                try:
                    self.rcnfclients[rcnfsrv] \
                    = dynamic_reconfigure.client.Client(rcnfsrv, timeout=5.0)
                    rospy.loginfo("Created client for {}.".format(rcnfsrv))
                except rospy.ROSException as e:
                    rospy.logerr(e)
                    rospy.signal_shutdown(e)
                    exit()

        
        # Get pose of the start node from the topological map and store 
        # for robot teleportation.
        self.model_state = ModelState()
        self.model_state.model_name = self.robot_name
        self.model_state.pose.orientation.w = 1
        for node in self.topo_map.nodes:
            if node.name == self.start_node:
                self.model_state.pose.position=node.pose.position
                break
            
        
        # Store the initial pose estimate = pose of the start node with a bit of noise.
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "/map"
        self.initial_pose.pose.pose = self.model_state.pose
        self.initial_pose.pose.covariance = [0.0043, -0.0006, 0.0, 0.0, 0.0, 0.0, -0.0006, 
                                        0.0013, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0003]
        
        
        # Robot model is at the start node before the test scenario starts.
        self.reset_robot()
        

    def map_callback(self, msg):
        """This function receives the Topological Map.
        """
        self.topo_map = msg
        self.rec_map = True
        

    def lookup_node(self, node_name):
        """Check that start/goal node exists.
        """        
        node_exists = False
        for node in self.topo_map.nodes:
            if node.name == node_name:
                node_exists = True
        try:
            assert node_exists == True
            rospy.loginfo("Found {}.".format(node_name))
        except AssertionError:
            rospy.logerr("{} doesn't exist.".format(node_name))
            rospy.signal_shutdown("{} doesn't exist.".format(node_name))
            exit()
            
    
    def lookup_route(self, start_node, goal_node):
        """Check that route between start node and goal node exists.
        """        
        route = self.router.search_route(start_node, goal_node)
        try: 
            assert route is not None
            rospy.loginfo("Route found.")
        except AssertionError:
            rospy.logerr("Route doesn't exist.")
            rospy.signal_shutdown("Route doesn't exist.")
            exit()
            
        
    def reset_robot(self):
        """Teleport the robot back to the start node, reset it's pose estimate and 
           clear costmaps.
        """
        self.set_model_state_client(self.model_state)
        rospy.sleep(1.0)
        self.init_pose_pub.publish(self.initial_pose)
        self.clear_costmaps_client()
        

    def run_scenario(self, params=None):
        """Run the test scenario (move from start node to goal node) with parameters 
           given by the GA and get fitness score = 1/time to complete
        """
        try:
            rospy.sleep(0.5)        
            print "\n"        
            rospy.loginfo("Running test scenario ...")
            
            # Reconfigure parameters
            if params is not None:
                for rcnfsrv in params.keys():
                    self.do_reconf(self.rcnfclients[rcnfsrv], params[rcnfsrv])
            
            time_1 = rospy.Time.now()
            self.topo_nav_client.send_goal_and_wait(self.topo_goal, self.max_wait_time)
            time_2 = rospy.Time.now()
            result = self.topo_nav_client.get_result()
            self.reset_robot()
            
            print result
            if result.success:
                t = (time_2-time_1).to_sec()
                print "Completed scenario in {} seconds.".format(t)
            else:
                t = self.max_wait_time.to_sec()
                print "Failed to complete scenario in maximum alloted time of {} seconds.".format(t)
                
        except rospy.ROSException:
            pass
        
        else:
            return 1.0/t # fitness
            
            
    def do_reconf(self, rcnfclient, params):
        """Reconfigure parameters.
        """
        try:
            rcnfclient.update_configuration(params)
        except rospy.ROSException as e:
            rospy.logerr(e)
            rospy.signal_shutdown(e)
            exit()
            
            
            
if __name__ == "__main__":
    
    rospy.init_node("scenario_server", anonymous=True, disable_signals=True)
    
    if len(sys.argv) < 3:
        rospy.loginfo("usage is optimise.py path_to_scenario_yaml path_to_parameters_yaml")
        exit()
    else:
        print sys.argv
        scenario = sys.argv[1]
        parameters = sys.argv[2]

    config_scenario = load_config_from_yaml(scenario)
    
    ss = scenario_server(config_scenario)  
    ss.run_scenario()
    ss.run_scenario()
#####################################################################################            
