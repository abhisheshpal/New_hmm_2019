#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rasberry_people_perception

import rospy

if __name__ == "__main__":
    rospy.init_node("simple_marvel_localiser", anonymous=True)

    sml = rasberry_people_perception.picker_marvel_localiser.PickerMarvelLocaliser()

    rospy.spin()
