#!/usr/bin/env python
"""
Created on Sat Feb 23 15:42:30 2019

@author: adam
"""
#####################################################################################
import rospy, sys, rosbag, numpy as np, rospkg
from rasberry_optimise.utils import *
from rasberry_optimise.metrics import *


if __name__ == "__main__":
    
    rospy.init_node("get_metrics", anonymous=True)    
    
    if len(sys.argv) < 4:
        rospy.loginfo("usage is get_metrics.py bag_filename path_to_bag_file centres_json")
        exit()
    else:
        print sys.argv
        print "\n"
        bag_file = sys.argv[1]
        data_dir = sys.argv[2]
        f_centres = sys.argv[3]
    
    rospack = rospkg.RosPack()
    resource_dir = rospack.get_path("rasberry_optimise") + "/resources/"    
    
    bag = rosbag.Bag(data_dir + "/" + bag_file)
    t, trajectory = get_trajectory(bag)
    bag.close()
    
    centres = load_data_from_yaml(resource_dir + f_centres)  
     
    rotation_cost = scorepath(np.array(trajectory))
    trajectory_length = get_trajectory_length(trajectory)
    path_error = get_path_error(trajectory, centres)
    
    rospy.loginfo("GETTING METRICS FROM " + bag_file + " ...")
    print "Completed scenario in {} seconds".format(t)
    print "Rotation cost = {}".format(rotation_cost)
    print "Length of trajectory = {} meters".format(trajectory_length)
    print "Path error = {} meters".format(path_error)
    print "\n"                
#####################################################################################