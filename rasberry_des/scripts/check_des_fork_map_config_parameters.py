#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.config_utils


if __name__ == "__main__":
    rospy.init_node("check_des_fork_map_config_parameters", anonymous=True)
    missing_params = rasberry_des.config_utils.check_fork_map_config()

    if len(missing_params) != 0:
        rospy.ROSException("Required DES configuration parameters missing")
    else:
        rospy.loginfo("All rasberry_des fork_map configuration parameters are present")
