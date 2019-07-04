# -*- coding: utf-8 -*-
"""
Created on 

@author: abhishesh
"""
"""
Parse the DES log data in Dataframes
"""
import yaml
from collections import defaultdict
import pandas as pd
from pprint import pprint


def parse_picker_states(picker_states):
    state_changes = picker_states['state_changes']

    state_changes_transposed = defaultdict(list)
    for state_change in state_changes:
        for key, value in state_change.items():
            state_changes_transposed[key].append(value)


    return pd.DataFrame(state_changes_transposed)

with open('/home/abhishesh/des_logs/Msingle_bed_P1_R0_Suniform_utilisation_1559653483487231_events.yaml') as f:
    data = yaml.load(f)


sim_details = data['sim_details']
id_df = []
for picker_states in sim_details['picker_states']:
	state_df = parse_picker_states(picker_states)

print ("state_df",state_df)