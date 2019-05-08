#!/bin/bash

WORKSPACE=rasberry_ws

cd /home/$USER
if [ -d $WORKSPACE ]; then echo "Abort. $WORKSPACE is already present"; exit 1; else mkdir -p $WORKSPACE/src; fi
cd $WORKSPACE/src
catkin_init_workspace
git clone --recursive https://github.com/LCAS/RASberry.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
echo "***Setup finished***"
