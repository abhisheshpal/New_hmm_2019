#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 01/10/2018
# ----------------------------------

import sys
import rospy
import rasberry_uv.uv_treatment


if __name__ == "__main__":
    rospy.init_node("simple_uv_treatment_node")
    # should also read a yaml file and read the row information
    # like start and finish nodes of each row and pass it as an argument

    if len(sys.argv) > 1:
        use_sim = sys.argv[1]
    else:
        use_sim = False

    uv_treat = rasberry_uv.uv_treatment.UVTreatment(use_sim=use_sim)
    rospy.on_shutdown(uv_treat.on_shutdown)

    while not rospy.is_shutdown():
        # TODO: Make an elegant trigger to start the uv treatment
        raw_input("Press ENTER to start the UV treatment")

        status = uv_treat.run()
        if not status:
            break
        rospy.loginfo("Ready to redo the UV treatment...")
