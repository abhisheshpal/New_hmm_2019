**rasberry_gazebo**
------------

Author: Adam Binch

email: adambinch@gmail.com

This package allows the generation of a Gazebo world with polytunnels and human models ('actors').
You can generate n polytunnels of arbitrary length positioned wherever you like in the Gazebo world.
The constraint is that the polytunnels are always aligned with the x-axis. Also, the polytunnel canopy is of a set width.
The package also permits the inclusion of n actors moving between user-specified waypoints.


# How to run:
To build a world open a terminal and do the following:

1. `roscd rasberry_gazebo/`
2. `chmod a+x ./scripts/generate_world.py`
3. `./scripts/generate_world.py --model_file ./config/gazebo/models_AB.yaml --actor_file ./config/gazebo/actors_AB.yaml`

If you have the Thorvald repo (https://github.com/LCAS/Thorvald) installed on your machine you can spawn the Thorvald robot model into the World you just built:

4. `roslaunch rasberry_gazebo thorvald_world_AB.launch`

If you do not then:

5. `roslaunch rasberry_gazebo world_AB.launch`

Note : you can subsitute `models_AB.yaml` and `actors_AB.yaml` in step 3 for your own config files.

An issue with the actors (which we will call 'type-1 actors') generated using the config file `./config/actors_AB.yaml` is that they cannot be controlled during simulation time.
Therefore another type of actor ('type-2 actor') is available in this package that can be controlled at runtime. These are robots with a human mesh, controlled with the standard 
`libgazebo_ros_planar_move` plugin. Two examples are spawned in the launch file `./launch/thorvald_world_AB.launch`. Here you can set the actor's name `actor_name` and starting pose.
After launching you can control the actor by issuing velcoity commands to the rostopic `/actor_name/cmd_vel`. Each actor also has an odometry topic `/actor_name/odom` and a laser scanner (with the visual turned off) with rostopic `/actor_name/scan`. 


# Info:
Open `./models_AB.yaml` for an example polytunnel configuration (there are a couple of other models that you can include as well as the polytunnels). 
Open `./actors_AB.yaml` for an example of how to include type-1 actors in the Gazebo world.

You will need to tell Gazebo to look for the models in `./models`. One way of doing this is to add the 
following to your bashrc file: `export GAZEBO_MODEL_PATH=~/path_to_rasberry_gazebo/rasberry_gazebo/models:$GAZEBO_MODEL_PATH`. 
For my machine: `export GAZEBO_MODEL_PATH=~/rasberry_ws/src/RASberry/rasberry_gazebo/models:$GAZEBO_MODEL_PATH`.

You will probably need a mid-range GPU or better to run this simulation properly.

Python package dependencies: argparse, xmltodict, rospkg, numpy, itertools, copy, json, yaml


# To do:
Implement standard move base navigation for the type 2 actors so that they can move between waypoints (nodes) in the topological map (the lasers referred to above will permit obstacle avoidance.)
Integrate this physical simulation with the discrete event simulation (https://github.com/LCAS/RASberry/tree/master/rasberry_des).


