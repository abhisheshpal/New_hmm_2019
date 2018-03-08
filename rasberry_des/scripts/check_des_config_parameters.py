#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import sys
import rasberry_des.config_utils


if __name__ == "__main__":
    if len(sys.argv) < 2:
        ns = "/rasberry_des_config/"
    else:
        ns = sys.argv[1] if sys.argv[1][-1] == '/' else sys.argv[1] + "/"

    rospy.init_node("check_des_config_parameters", anonymous=True)

    missing_params = rasberry_des.config_utils.check_des_config(ns)

    if len(missing_params) != 0:
        rospy.ROSException("Required DES configuration parameters missing")
    else:
        rospy.loginfo("All rasberry_des configuration parameters are present")
