#!/bin/bash

SESSION=$USER
DISPLAY=0

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'base'
tmux new-window -t $SESSION:2 -n 'localisation'
tmux new-window -t $SESSION:3 -n 'move_base'
tmux new-window -t $SESSION:4 -n 'topo_nav'
tmux new-window -t $SESSION:5 -n 'rviz'
tmux new-window -t $SESSION:6 -n 'marvel'
tmux new-window -t $SESSION:7 -n 'coordination'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "echo 'robot localisation'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_bringup robot_bringup.launch robot_model:=$(rospack find rasberry_bringup)/config/norway_robot.yaml model_extras:=$(rospack find rasberry_bringup)/urdf/norway_robot_sensors.xacro simple_sim:=true world_name:=norway_poles with_actors:=false"

tmux select-window -t $SESSION:2
tmux send-keys "echo 'robot localisation'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base amcl.launch"

tmux select-window -t $SESSION:3
tmux send-keys "echo 'move_base actions for the robot'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_dwa.launch"

tmux select-window -t $SESSION:4
tmux send-keys "echo 'topological navigation actions for the robot'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_navigation topological_navigation_robot.launch move_base_reconf_service:=DWAPlannerROS"

tmux select-window -t $SESSION:5
tmux send-keys "echo 'rviz'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rosrun rviz rviz -d $(rospack find rasberry_bringup)/resources/topological_navigation.rviz"

tmux select-window -t $SESSION:6
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "echo 'mimic marvelmind_nav/hedge_rcv_bin or run that node if available'" C-m
tmux send-keys "echo 'rosrun marvelmind_nav hedge_rcv_bin'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rostopic pub -r 10 /hedge_pos_a marvelmind_nav/hedge_pos_a -- '1' '0' '11.0' '5.7' '0.0' '0'"
tmux select-pane -t 1
tmux send-keys "echo 'mimic marvelmind_nav/hedge_rcv_bin or run that node if available'" C-m
tmux send-keys "echo 'rosrun marvelmind_nav hedge_rcv_bin'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rostopic pub -r 10 /hedge_pos_a marvelmind_nav/hedge_pos_a -- '5' '0' '16.0' '4.2' '0.0' '0'"

tmux select-window -t $SESSION:7
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "echo 'rosbridge'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_coordination coordination.launch"
tmux select-pane -t 1
tmux send-keys "echo 'rosduct to server'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_coordination robot_websocket_adapter.launch robot_name:=thorvald_003 rosbridge_ip:=10.8.0.4 rosbridge_port:=9090"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
