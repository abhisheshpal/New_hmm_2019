#!/bin/bash

SESSION=$USER
DISPLAY=0

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo_topo_man'
tmux new-window -t $SESSION:2 -n 'map_server'
tmux new-window -t $SESSION:3 -n 'rviz'
tmux new-window -t $SESSION:4 -n 'people'
tmux new-window -t $SESSION:5 -n 'coordination'
tmux new-window -t $SESSION:6 -n 'scheduler'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "echo 'mongodb'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch mongodb_store mongodb_store.launch db_path:=$1"
tmux select-pane -t 1
tmux send-keys "echo 'topological map server'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_navigation topological_map_manager_central.launch map:=norway_poles"

tmux select-window -t $SESSION:2
tmux send-keys "echo '2D map server'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base map_server.launch map:=$(rospack find rasberry_navigation)/maps/norway_poles.yaml"

tmux select-window -t $SESSION:3
tmux send-keys "echo 'rviz'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rosrun rviz rviz -d $(rospack find rasberry_bringup)/resources/topological_navigation_norway_poles.rviz"

tmux select-window -t $SESSION:4
tmux send-keys "echo 'marvelmind localiser'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rosrun rasberry_people_perception simple_marvel_localiser.py"

tmux select-window -t $SESSION:5
tmux send-keys "echo 'rosbridge'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_coordination coordination.launch"

tmux select-window -t $SESSION:6
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "echo 'callarobot websocket client'" C-m
tmux send-keys "DISPLAY=:$DISPLAY WEBSOCKET_URL='wss://lcas.lincoln.ac.uk/car/ws' python $(rospack find rasberry_coordination)/callarobot/ws_client.py"
tmux select-pane -t 1
tmux send-keys "echo 'task scheduler'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rosrun rasberry_coordination simple_task_executor_node.py $(rospack find rasberry_coordination)/config/map_config_norway_poles.yaml"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
