#!/usr/bin/env python
from __future__ import division
import yaml, tf


def load_data_from_yaml(filename):
    with open(filename,'r') as f:
        return yaml.load(f)

        
def save_data_to_yaml(filename, data):
    with open(filename,'w') as f:
        return yaml.dump(data, f)


def make_param_list(config_params, individual):
    
    params = []
    for config_param, val in zip(config_params, individual):
        param = {}
        param["name"] = config_param["name"]
        param["ns"] = config_param["ns"]
        param["type"] = config_param["type"]
        param["value"] = val
        params.append(param)
            
    return params
    
    
def apply_constraints(params):

    for param in params:
        
        # If optimising teb local planner then constrain `dt_hysteresis` to be 10% of `dt_ref`
        if param["ns"] == "move_base/TebLocalPlannerROS" \
        and param["name"] == "dt_ref":
            dt_hysteresis = {}
            dt_hysteresis["name"] = "dt_hysteresis"
            dt_hysteresis["ns"] = "move_base/TebLocalPlannerROS"
            dt_hysteresis["type"] = "double"
            dt_hysteresis["value"] = 0.1 * float(param["value"])
            params.append(dt_hysteresis)
            
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