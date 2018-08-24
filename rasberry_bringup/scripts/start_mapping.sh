#!/bin/bash

SESSION=$USER
DISPLAY=0

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'bringup'
tmux new-window -t $SESSION:2 -n 'hokuyo'
tmux new-window -t $SESSION:3 -n 'mapping'
tmux new-window -t $SESSION:4 -n 'map_saver'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux select-pane -t 0
tmux send-keys "echo 'robot bringup'" C-m
tmux send-keys "DISPLAY=:0 roslaunch rasberry_bringup robot_bringup.launch robot_model:=/home/thorvald/thorvald_ws/src/RASberry/rasberry_bringup/config/uk_robot_007.yaml model_extras:=/home/thorvald/thorvald_ws/src/RASberry/rasberry_bringup/urdf/uk_robot_007_sensors.xacro simple_sim:=false"

tmux select-window -t $SESSION:2
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "echo 'robot hokuyo'" C-m
tmux send-keys "roslaunch rasberry_bringup hokuyo.launch"
tmux select-pane -t 1
tmux send-keys "echo 'robot xsens'" C-m
tmux send-keys "roslaunch rasberry_bringup xsens_driver.launch"

tmux select-window -t $SESSION:3
tmux send-keys "echo 'robot mapping'" C-m
tmux send-keys "rosrun gmapping slam_gmapping"

tmux select-window -t $SESSION:4
tmux send-keys "echo '2D map saver'" C-m
tmux send-keys "cd Documents" C-m
tmux send-keys "rosrun map_server map_saver"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
