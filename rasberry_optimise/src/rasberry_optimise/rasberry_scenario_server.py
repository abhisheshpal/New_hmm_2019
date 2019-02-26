#!/usr/bin/env python
from __future__ import division
import rospy, actionlib, dynamic_reconfigure.client, sys, tf, rospkg
import numpy as np
import message_filters
from copy import copy
from gazebo_msgs.srv import SetModelState 
from gazebo_msgs.msg import ModelState, ContactState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from topological_navigation import route_search
from strands_navigation_msgs.msg import TopologicalMap
from std_srvs.srv import Empty
from rasberry_optimise.metrics import *
from rasberry_optimise.utils import *


class scenario_server(object):
    """scenario_server class definition.    
    """
    
    def __init__(self, config_scenario, rcnfsrvs=None):
        """
        Keyword arguments:
        config   -- dictionary containing the configuration parameters for running a 
                    test scenario.
        rcnfsrvs -- list of reconfigure services to make clients for (optional).
        """
        
        rospy.loginfo("Initialising scenario server ...")        
        
        # Get config for test scenario.
        self.start_node = config_scenario["start_node"]
        self.goal_node = config_scenario["goal_node"]
        self.robot_name = config_scenario["robot_name"]
        self.max_wait_time = rospy.Duration(config_scenario["max_wait_time"])
        
        if "coords_file" in config_scenario.keys():
            self.coords_file = config_scenario["coords_file"]
            rospack = rospkg.RosPack()
            base_dir = rospack.get_path("rasberry_optimise")    
            coords_file = base_dir + "/resources/" + self.coords_file
            self.coords = load_data_from_json(coords_file)
        else:
            self.coords_file = None
            
        
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
        if rcnfsrvs is not None:
            rospy.loginfo("Creating reconfigure clients ...")
            self.rcnfclients = {}
            for rcnfsrv in rcnfsrvs:
                try:
                    self.rcnfclients[rcnfsrv] \
                    = dynamic_reconfigure.client.Client(rcnfsrv, timeout=5.0)
                    rospy.loginfo("Created client for {}".format(rcnfsrv))
                except rospy.ROSException as e:
                    rospy.logerr(e)
                    rospy.signal_shutdown(e)
                    exit()

        
        # Get pose of the start node from the topological map and store 
        # for robot teleportation.
        self.model_state = ModelState()
        self.model_state.model_name = self.robot_name
        for node in self.topo_map.nodes:
            if node.name == self.start_node:
                self.model_state.pose=node.pose
                break
            
        
        # Store the initial pose estimate = pose of the start node with a bit of noise.
        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "/map"
        self.initial_pose.pose.pose = self.model_state.pose
        self.initial_pose.pose.covariance = [0.006635127796202456, 0.0006228153697929173, 
                                             0.0, 0.0, 0.0, 0.0, 0.0006228153697929173, 
                                             0.004126767460876479, 0.0, 0.0, 0.0, 0.0, 
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                             0.0002255732129925828]
        
        
        # Robot model is at the start node before the test scenario starts.
        self.reset_robot()
        self.robot_poses = []
        self.robot_poses_filtered = []
        self.amcl_poses_filtered = []
        

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
        rospy.sleep(3.0)
        self.init_pose_pub.publish(self.initial_pose)
        self.clear_costmaps_client()
        

    def run_scenario(self, params=None):
        """Run the test scenario (move from start node to goal node) 
           and get time to complete.
        """
        try:
            rospy.sleep(1.0)
            print "\n"
            rospy.loginfo("Running test scenario ...")
            
            # Reconfigure parameters
            if params is not None:
                for rcnfsrv in params.keys():
                    self.do_reconf(self.rcnfclients[rcnfsrv], params[rcnfsrv])
                
            # For getting the robot's trajectory
            rp_sub = rospy.Subscriber("/odometry/gazebo", Odometry, self.rp_callback)
            rp_sub_filtered = message_filters.Subscriber("/odometry/gazebo", Odometry)
            amcl_sub_filtered = message_filters.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
            
            time_sync = message_filters.ApproximateTimeSynchronizer([rp_sub_filtered, amcl_sub_filtered], 10, 0.1, allow_headerless=True)
            time_sync.registerCallback(self.rp_filtered_callback)
            
            # Listen for collisions between robot model and other objects.
            self.collided = False
            global contact_sub
            contact_sub = rospy.Subscriber("/collision_data_throttled", ContactState , self.contact_callback)              
            
            time_1 = rospy.Time.now()
            self.topo_nav_client.send_goal_and_wait(self.topo_goal, self.max_wait_time)
            time_2 = rospy.Time.now()
            result = self.topo_nav_client.get_result()
            
            rp_sub.unregister() 
            rp_sub_filtered.unregister() 
            amcl_sub_filtered.unregister() 

            trajectory = copy(self.robot_poses)
            del self.robot_poses[:]
            trajectory_ground_truth = copy(self.robot_poses_filtered)
            del self.robot_poses_filtered[:]
            trajectory_amcl = copy(self.amcl_poses_filtered)
            del self.amcl_poses_filtered[:]
            
            self.reset_robot()
            
            print result
            if result.success:
                
                # Get metrics
                t = (time_2-time_1).to_sec()
                cost_dollars = scorepath(np.array(trajectory))
                trajectory_length = get_trajectory_length(trajectory)
                
                if self.coords_file is not None:
                    dist_from_coords = get_dist_from_coords(self.coords, trajectory)
                else:
                    dist_from_coords = 0.0
                    
                pose_error, position_error, orientation_error = get_localisation_error(trajectory_ground_truth, trajectory_amcl)
                
                print "Completed scenario in {} seconds".format(t)
                print "Rotation cost = {} dollars".format(cost_dollars)
                print "Length of trajectory = {} meters".format(trajectory_length)
                print "Sum squared distance from coordinates = {} meters squared".format(dist_from_coords)
                print "Mean pose error = {}".format(pose_error)                
                print "Mean position error = {} meters".format(position_error) 
                print "Mean orientation error = {} degrees".format(orientation_error) 
                
            else:
                t = self.max_wait_time.to_sec()
                cost_dollars = -10e5
                trajectory_length = 10e5
                dist_from_coords = 10e5
                pose_error = 10e5
                print "Failed to complete scenario in maximum alloted time of {} seconds".format(t)
                
        except rospy.ROSException:
            pass
        
        else:
            metrics = (t, cost_dollars, trajectory_length, dist_from_coords, pose_error)
            return metrics, trajectory
            
            
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
            
            
    def rp_callback(self, msg):
        robot_pose = self.get_pose(msg) 
        self.robot_poses.append(robot_pose)        
            
            
    def rp_filtered_callback(self, msg_rp, msg_amcl):
        robot_pose = self.get_pose(msg_rp) 
        amcl_pose = self.get_pose(msg_amcl) 
        self.robot_poses_filtered.append(robot_pose)
        self.amcl_poses_filtered.append(amcl_pose)
            
            
    def get_pose(self, msg):
        """Get robot poses and append them to a list to form the robot's trajectory.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        xq = msg.pose.pose.orientation.x
        yq = msg.pose.pose.orientation.y
        zq = msg.pose.pose.orientation.z
        wq = msg.pose.pose.orientation.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([xq, yq, zq, wq])
        return [x, y, yaw]
        

    def contact_callback(self, msg):
        """Cancel nav goal if collision detected. Excludes collisions with the 
           ground plane.
        """
        col1_name = msg.collision1_name
        col2_name = msg.collision2_name
        if self.robot_name in col1_name or self.robot_name in col2_name:
            if col1_name != "grass_ground_plane::link::collision" \
            and col2_name != "grass_ground_plane::link::collision":
                if not self.collided:
                    contact_sub.unregister()
                    rospy.logwarn("COLLISION DETECTED")
                    self.topo_nav_client.cancel_goal()
                    self.collided = True


            
            
if __name__ == "__main__":
    
    rospy.init_node("scenario_server", anonymous=True, disable_signals=True)
    
    if len(sys.argv) < 2:
        rospy.loginfo("usage is optimise.py path_to_scenario_yaml")
        exit()
    else:
        print sys.argv
        scenario = sys.argv[1]

    config_scenario = load_data_from_yaml(scenario)
    
    ss = scenario_server(config_scenario)  
    ss.run_scenario()
#####################################################################################