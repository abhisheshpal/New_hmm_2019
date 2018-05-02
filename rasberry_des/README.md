**rasberry_des**
------------
A rospackage for running discrete event simulation of a strawberry farm, pickers, robots and a basic job allocation processes using SimPy.

A topological map is created and stored in mongodb with corresponding topics and services launched using the topological navigation nodes.

# Nodes:
  1. `des.py`
    Based on the configuration parameters set, run either a simpy or wall-clock discrete event simulation.
  2. `check_des_config_parameters.py`
    For checking whether all configuration parameters required for the DES are set.
  3. `check_des_fork_map_config_parameters.py`
    For checking whether all configuration parameters required for generating the fork_map are set.
  4. `fork_map_generator.py`
    Node to generate `fork_map`, a topological map in the mongodb. 
  5. `gazebo_polytunnel_map_generator.py`
    Node to generate topological map for the gazebo configuration in `rasberry_gazebo`. The `config_file` to be used with this node is different from the `des_config` file.

# Launch files
  1. `map_broadcaster.launch`
  2. `des.launch`

# How to run

1. Launch the `mongodb_store` nodes
  `roslaunch mongodb_store mongodb_store.launch db_path:=<path_to_mongo_db_storage_dir>`
2. Launch some preliminary nodes
  * `roslaunch rasbderry_des map_broadcaster.launch [map:="<path_to_metric_map.yaml>]`
  * Check whether the pointset you want is loaded in mongodb
    `rosrun topological_utils list_maps`
    If the pointset is loaded, go to Step 3.
    If the pointset is not loaded and you don't have a tmap/yaml file of the pointset, go to Step 4.
    If the pointset is not loaded and you have a tmap/yaml file of the pointset, go to Step 5.
3. * Launch the `topological_navigation` nodes. 
     `roslaunch topological_navigation topological_navigation.launch map:="<tplg_map_dataset_name>`
   * Go to Step 6
4. If no maps are available: generate a topological fork_map (in future, it can be done through rviz)
  * Launch the `topological_navigation` nodes. This will create a `topological_map` with the given name in mongodb, and nodes, edges added to the map would be with the name of the map as `pointset`.
    `roslaunch topological_navigation topological_navigation_empty_map.launch map:="<tplg_map_dataset_name>"`
  * Edit the `config/des_config.yaml` with the required configuration parameters for the discrete event simulation, or prepare a yaml file with the same keys.
  * Check whether all des fork_map config parameters are loaded by running
    `rosrun rasberry_des check_des_fork_map_config_parameters.py <path_to_des_config_file>`
  * Generate the map (assuming only fork_map at this stage).
    `rosrun rasberry_des fork_map_generator.py ns:="/"`
  * Now all nodes are in the mongodb. To visualise them in rviz,
    * `rosrun rviz rviz`
    * Add `Map` elemet with topic `map`
    * Export map to a .tmap file. 
      `rosrun topological_utils map_export.py <tplg_map_dataset_name> <tplg_map_name.tmap>`
      Here, the `<tplg_map_dataset_name>` is same as the one used earlier. All nodes were added to this dataset.
    * Export the topological map to a yaml file
      `rosrun topological_utils tmap_to_yaml.py <infile.tmap> <outfile.yaml> <tplg_map_dataset_name> <tplg_map_name>`
    * Until now /topological_map topic is not published and the topological map cannot be visualised in rviz. To update the topological map nodes and edges with the generated nodes and edges, 
      `rosrun topological_utils topological_map_update.py`
      Add `MarkerArray` element with topic `/topological_map_visualisation`
      The `/topological_map` topic is also being published
    * Go to Step 6
5. If you already have a map (as a waypoint, yaml or tmap file), load topological_navigation nodes
  * Edit the `config/des_config.yaml` with the required configuration parameters for the discrete event simulation, or prepare a yaml file with the same keys.
  * If the topological map nodes are not in the mongodb, store them in mongodb first
    * if only a waypoint file is available, create tmap and yaml files and load any of them to the mongodb.
      `rosrun topological_utils waypoints_to_yaml_tmap.py <input_file.txt> <outfile> <tplg_map_dataset_name> <tplg_map_name> [max_dist_connect]`
    * from a yaml file
      `rosrun topological_utils load_yaml_map.py [-h] [--pointset POINTSET] [-f] [--keep-alive] <in_mapfile.yaml>`
    * from a tmap file
      `rosrun topological_utils insert_map.py <infile.tmap> <tplg_map_dataset_name> <tplg_map_name>`
  * Launch the `topological_navigation` nodes. 
    `roslaunch topological_navigation topological_navigation.launch map:="<tplg_map_dataset_name>`
  * Update the ros parameters corresponding to the topological_map
    * Go to Step 6
6. Discrete event simulation
  * If not done already, edit the `config/des_config.yaml` with the required configuration parameters for the discrete event simulation, or prepare a yaml file with the same keys.
  * Make sure all required DES configuration parameters are set.
    `rosrun rasberry_des check_des_config_parameters.py <path_to_des_config_file>`
    If any parameters are not set, edit the `des_config.yaml` (or your custom configuration file).
  * Run the `des.py` discrete event simulation. 
    `roslaunch rasberry_des des.launch config_file:=<path_to_des_config_file>`
    or
    `rosrun rasberry_des des.py <path_to_des_config_file>`

# Main classes:
`TopologicalForkMap` in `topo.py`
`Farm` in `farm.py`
`Picker` in `picker.py`
`Robot` in `robot.py`
`VisualiseAgents` in `visualise.py`

# Info:
A fork like topological map created and stored in mongodb is accessed by `TopologicalForkMap`, which in turn used by all other agent classes. This map consists of one head lane and many topological navigation rows. The length of each row, node distances and yield per node distance can be different per row - this can be configured in the `des_config.yaml` file. The number of pickers and robots can also be changed along with their characteristic features such as picking rate, transportation rate, loading and unloading time etc can be configured through `des_config.yaml`. Three possible scheduling policies are implemented now - `"lexographical", "shortest_distance", "utilise_all"`, which can be configured in `des.py`. Visualisation and logging can be enabled or disabled in `des.py`.

# Known Issues:
  1. `env.step` is used to step through the DES. 
     - some delay between the two clocks at the start. SimPy clock does not progress until the first `env.step`, but the all required process should be initialised before that.
     - Some delay (1-2 ms in ros/real-time clock) is observed between multiple events scheduled at the same simulation time.
     - However, chances of having multiple events scheduled at the same instance will be very small except at the start.
     - This delay does not seem to build up over time.

# TODO:
  1. More complex map may be defined in future using this. For example, long rows with two head lanes at both ends. A row may be assigned to two pickers who will start from either end, with associated local storage on the head lane on that side.

# These may not be implemented as part of DES
  to make the simulations faster.
  1. ROS based communication between agents
  2. Simulate humans and robots in gazebo, which move according to the actions in the DES.
  3. A definition of a scheduling interface (maybe inspired by strands, ROS msgs, and ROS action definitions to

    request tasks
    cancel tasks
    monitor task execution


