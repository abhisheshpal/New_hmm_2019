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
SUCCESS=$?
if [ $SUCCESS == 0 ]; then echo -e "\n***Setup finished***\n"; source devel/setup.bash; else echo -e "\nSetup failed!!!\n"; fi