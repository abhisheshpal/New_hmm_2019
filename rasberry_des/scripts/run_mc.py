#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import numpy
import os
import sys

import rasberry_des.event_log_processor


class MC(object):
    def __init__(self, logs_dir, ):
        self.logs_dir = os.path.abspath(logs_dir)

        self.log_data = None
        self.n_iter = 0
        self.n_pickers = 0
        self.picker_ids = []

        self.get_logs_data()

    def get_logs_data(self, ):
        """assumes the event log files are in yaml format and all the logs to be processed
        for all the trials in a single directory
        """
        for f_name in os.listdir(self.logs_dir):
            if not(os.path.isfile(f_name) and f_name.startswith("M") and f_name.endswith(".yaml")):
                continue
            log_data = rasberry_des.event_log_processor.get_log_data(f_name)
            if log_data is not None:
                self.log_data.append(log_data)
        if len(self.log_data) > 0:
            self.n_iter = len(self.log_data)
            self.picker_ids = rasberry_des.event_log_processor.get_picker_ids(self.log_data[0])
            self.n_pickers = len(self.picker_ids)

    def run_mc(self, picker_id, curr_state, curr_state_init_time, time_now):
        pass