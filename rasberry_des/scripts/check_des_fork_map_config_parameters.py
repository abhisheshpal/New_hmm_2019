#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import sys
import rospy
import rasberry_des.config_utils


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        raise Exception("Not enough arguments. Pass 'path to configuration file' as an argument.")
    else:
        config_file = sys.argv[1]

    rospy.init_node("check_des_fork_map_config_parameters", anonymous=True)
    missing_params = rasberry_des.config_utils.check_fork_map_config(config_file)
    rospy.loginfo(missing_params)
    rospy.sleep(1)