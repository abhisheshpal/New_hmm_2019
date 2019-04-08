#!/usr/bin/env python
from __future__ import division
import yaml, json, tf


def load_data_from_yaml(filename):
    with open(filename,'r') as f:
        return yaml.load(f)

        
def save_data_to_yaml(filename, data):
    with open(filename,'w') as f:
        return yaml.dump(data, f)

        
def load_data_from_json(filename):
    with open(filename) as f:
        return json.load(f)
        
        
def save_data_to_json(filename, data):
    with open(filename, 'w') as f:
        json.dump(data, f)
        

teb_rcnfsrv = "/move_base/TebLocalPlannerROS"        
rcnfsrv1 = "/move_base/local_costmap/local_inflation_layer"
rcnfsrv2 = "/move_base/global_costmap/global_inflation_layer"
param1 = "inflation_radius"
param2 = "cost_scaling_factor"        

def make_param_dict(config_params, individual):

    params = {}
    count = 0
    rcnfsrvs = config_params.keys()
    for i, rcnfsrv in enumerate(rcnfsrvs):
        
        params[rcnfsrv] = {}
        param_names = config_params.values()[i].keys()
        for param_name in param_names:
            
            if config_params.values()[i][param_name]['type'] == "bool":
                
                if individual[count] == 1:
                    params[rcnfsrv][param_name] = 'true'
                else:
                    params[rcnfsrv][param_name] = 'false'
                    
            else:
                params[rcnfsrv][param_name] = individual[count]
                
            count += 1
            
    # If optimising teb local planner then constrain `dt_hysteresis` to be 10% of `dt_ref` 
    if teb_rcnfsrv in rcnfsrvs:
        if "dt_ref" in params[teb_rcnfsrv].keys():
            params[teb_rcnfsrv]["dt_hysteresis"] = 0.1 * float(params[teb_rcnfsrv]["dt_ref"])
        
    # Constrain global inflation parameters to equal local inflation parameters       
    if rcnfsrv1 in rcnfsrvs and rcnfsrv2 not in rcnfsrvs:
        params[rcnfsrv2] = {}

        if param1 in params[rcnfsrv1].keys():
            params[rcnfsrv2][param1] = params[rcnfsrv1][param1]

        if param2 in params[rcnfsrv1].keys():
            params[rcnfsrv2][param2] = params[rcnfsrv1][param2]
            
    return params
    
    
def get_trajectory(bag):
    ts = []
    trajectory = []
    for topic, msg, t in bag.read_messages(topics=["/amcl_pose"]):
        ts.append(t.to_sec())
        trajectory.append(get_pose(msg))            
    return ts[-1] - ts[0], trajectory
    

def get_pose(msg):
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
#####################################################################################