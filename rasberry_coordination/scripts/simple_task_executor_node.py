#!/usr/bin/env python

import sys

import rospy

import rasberry_coordination.coordinator
import rasberry_coordination.picker_state_monitor

import rasberry_des.config_utils

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print """not enough arguments are passed. correct usage is
        rosrun rasberry_coordination simple_task_executor_node.py config_file.yaml
        """
    else:
        print sys.argv
        config_file = sys.argv[1]
        config_data = rasberry_des.config_utils.get_config_data(config_file)
        config_keys = rasberry_des.config_utils.get_config_keys(config_file)

        # check for required parameters
        req_params = ["base_station_node", "local_storage_node", "charging_station_node", "robot_ids"]

        for key in config_keys:
            if key in req_params:
                req_params.remove(key)

        if len(req_params) != 0:
            raise Exception("not all required keys are set in the config file")
        elif config_data["robot_ids"].__class__ != list:
            raise Exception("robot_ids should be a list in the config file")
        elif len(config_data["robot_ids"]) == 0:
            raise Exception("robot_ids should not be an empty list in the config file")

        base_station = config_data["base_station_node"] # list of local storage nodes
        local_storage = config_data["local_storage_node"] # list of local storage nodes
        charging_node = config_data["charging_station_node"]
        robot_ids = config_data["robot_ids"]
        if "unified" in config_keys:
            unified = config_data["unified"]
        else:
            unified = False

        rospy.init_node('simple_task_coordinator', anonymous=False)

        # initialise the coordinator and internally all robots
        coordinator = rasberry_coordination.coordinator.Coordinator(robot_ids=robot_ids,
                                                          local_storage=local_storage,
                                                          charging_node=charging_node,
                                                          base_station=base_station,
                                                          unified=unified)
        rospy.on_shutdown(coordinator.on_shutdown)
        rospy.sleep(1) # give a second to let everything settle

        # picker_monitor after coordinator
        picker_monitor = rasberry_coordination.picker_state_monitor.PickerStateMonitor()

        coordinator.run()

        rospy.spin()
