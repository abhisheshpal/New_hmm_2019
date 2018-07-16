#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n 'move_base'
tmux new-window -t $SESSION:4 -n 'navigation'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=$1"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch rasberry_bringup rasberry_simulation.launch with_actors:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch rasberry_move_base move_base_teb.launch map:=$(rospack find rasberry_gazebo)/maps/riseholme_sim.yaml"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:0 roslaunch topological_navigation topological_navigation.launch map:=polys move_base_reconf_service:=TebLocalPlannerROS"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
