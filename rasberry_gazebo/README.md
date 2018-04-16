Author: Adam Binch
email: adambinch@gmail.com

This package allows the generation of a Gazebo world with polytunnels and human models ('actors').
You can generate n polytunnels of arbitrary length positioned wherever you like in the Gazebo world.
The constraint is that the polytunnels are always aligned with the x-axis. Also, the polytunnel canopy is of a set width.
The package also permits the inclusion of n actors moving between user specified waypoints.

Open ./models_AB.yaml for an example polytunnel configuration (there are a couple of other models that you can include as well as the polytunnels).
Open ./actors_AB.yaml for an example of how to include moving actors in the Gazebo world.
You can make your own .yaml files.

To build a world open a terminal and do the following:
$ roscd rasberry_gazebo/
$ chmod a+x ./scripts/generate_world.py
$ ./scripts/generate_world.py --model_file ./config/models_AB.yaml --actor_file ./config/actors_AB.yaml
$ roslaunch rasberry_gazebo thorvald_world_AB.launch
Note : you can subsitute 'models_AB.yaml' and 'actors_AB.yaml' for your own config files.

You will need to tell gazebo to look for the models in ./models. One way of doing this is to add the 
following to your bashrc file:
export GAZEBO_MODEL_PATH=~/path_to_rasberry_gazebo/rasberry_gazebo/models:$GAZEBO_MODEL_PATH
For my machine:
export GAZEBO_MODEL_PATH=~/rasberry_ws/src/RASberry/rasberry_gazebo/models:$GAZEBO_MODEL_PATH

You will probably need a mid range GPU or better to run this simulation properly.

I am currently spawing Thorvald into the world using the following (possibly outdated) method. 
Open a second terminal:
$ roslaunch thorvald_bringup thorvald_ii_4wd4ws_slim_sim.launch
Open a third terminal:
$ rosrun gazebo_ros spawn_model -urdf -param /robot_description -model thorvald_ii
