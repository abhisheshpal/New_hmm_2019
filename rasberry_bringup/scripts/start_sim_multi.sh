#!/bin/bash

SESSION=$USER
DISPLAY=1

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n 'map_server'
tmux new-window -t $SESSION:4 -n 't1-move_base'
tmux new-window -t $SESSION:5 -n 't2-move_base'
#tmux new-window -t $SESSION:6 -n 'navigation'
#tmux new-window -t $SESSION:7 -n 'rviz'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:$DISPLAY roslaunch mongodb_store mongodb_store.launch db_path:=$1"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_bringup rasberry_simulation_multi.launch with_actors:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base map_server.launch map:=$(rospack find rasberry_gazebo)/maps/riseholme_sim.yaml"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_teb_multi.launch robot_name:=thorvald_001"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_teb_multi.launch robot_name:=thorvald_002"

#tmux select-window -t $SESSION:6
#tmux send-keys "DISPLAY=:$DISPLAY roslaunch topological_navigation topological_navigation.launch map:=riseholme move_base_reconf_service:=TebLocalPlannerROS"

#tmux select-window -t $SESSION:7
#tmux send-keys "DISPLAY=:$DISPLAY  rosrun rviz rviz -d $(rospack find rasberry_bringup)/resources/topological_navigation.rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
