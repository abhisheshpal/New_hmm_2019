#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import rospy
import rasberry_des.picker



class PickerMimic(rasberry_des.picker.Picker):
    """PickerMimic class to mimic long picking operations
    """
    def __init__(self, picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, verbose=False):
        """Initialise PickerMimic class
        """
        super(PickerMimic, self).__init__(picker_id, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, unloading_time,
                 env, topo_graph, robots, verbose)