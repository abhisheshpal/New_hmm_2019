**rasberry_des**
------------
A rospackage for running discrete event simulation of a strawberry farm, pickers and a basic job allocation processesusing SimPy. It does not rely on any ros related functionalities yet, other than initialising the node.

# Nodes:
  A topological map is created and stored in mongodb. However, the Farm and Picker objects still use a map created with the Node and Graph definitions in `rasberry_des.task`. This has to be modified with the functionalities in the `strands_navigation`.

  1. `pickers_only.py`
    Based on the configuration parameters set, run either a simpy or wall-clock discrete event simulation.
  2. `check_des_config_parameters.py`
    For checking whether all configuration parameters required for the des are set.
  3. `check_des_fork_map_config_parameters.py`
    For checking whether all configuration parameters required for generating the fork_map are set.
  4. `fork_map_generator.py`
    Node to generate `fork_map`, a topological map in the mongodb. 
  5. `tf2_broadcaster.py`

# How to run

1. Launch the `mongodb_store` nodes
`roslaunch mongodb_store mongodb_store.launch db:=<path_to_mongo_db_storage_dir>`
2. If no maps are available: generate a topological fork_map (in future, it can be done through rviz)
  * Edit the `rasberry_des/config/des_config.yaml` with the required configuration parameters for the discrete event simulation, or prepare a yaml file with the same keys.
  * Launch the `topological_navigation` nodes. This will create a `topological_map` with the given name in mongodb, and nodes, edges added to the map would be with the name of the map as `pointset`.
    `roslaunch rasberry_des topological_navigation_empty_map.launch topo_map:="<tplg_map_dataset_name> [map:="<path_to_metric_map.yaml>" config_file:="<path_to_des_config_file.yaml>"]`
  * Check whether all des fork_map config parameters are loaded by running
    `rosrun rasberry_des check_des_fork_map_config_parameters.py [<namespace>]`
  * Generate the map (assuming only fork_map at this stage).
    `rosrun rasberry_des fork_map_generator.py [<namespace>]`
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
3. If you already have a map, load topological_navigation nodes
  * Edit the `rasberry_des/config/des_config.yaml` with the required configuration parameters for the discrete event simulation, or prepare a yaml file with the same keys.
  * If the topological map nodes are not in the mongodb, store them in mongodb
    * if only a waypoint file is available, create tmap and yaml files from that.
      `rosrun topological_utils waypoints_to_yaml_tmap.py <input_file.txt> <outfile> <tplg_map_dataset_name> <tplg_map_name> [max_dist_connect]`
    * from a yaml file
      `rosrun topological_utils load_yaml_map.py [-h] [--pointset POINTSET] [-f] [--keep-alive] <in_mapfile.yaml>`
    * from a tmap file
      `rosrun topological_utils insert_map.py <infile.tmap> <tplg_map_dataset_name> <tplg_map_name>`
  * Launch the `topological_navigation` nodes. 
    `roslaunch rasberry_des topological_navigation.launch topo_map:="<tplg_map_dataset_name> [map:="<path_to_metric_map.yaml>" config_file:="<path_to_des_config_file.yaml>"]`
  * Update the ros parameters corresponding to the topological_map
4. Discrete event simulation
  * Complete Step 2 or 3. Make sure all required des configuration parameters are set
    `rosrun rasberry_des check_des_config_parameters.py [<namespace>]`
    If any parameters are not set, edit the `des_config.yaml` file mentioned in Step 2 or 3 and relaunch the `topological_navigation` nodes.
  * `roslaunch rasberry_des pickers_only.launch [namespace]`

# Main classes:
A `Farm` class is defined in `farm.py`:
  1. A topological fork like map, consisting of a head lane and many rows. The map uses `Graph` and `Node` classes in `topo.py`. These classes contain some methods like `a_star` to get the shortest path between two nodes.
  2. A local storage at the centre of the head lane. Ideally there should be more than one local storage nodes.
  3. A variable number of nodes along each row. The yield at each node is mapped as a logistic distribution.
  4. A group of pickers who report to work.
  5. A `scheduler_monitor` method, which is run as a process in SimPy to monitor the row_completion events from the assigned pickers and to allocate the unallocated rows to free pickers. This process ends when there are no more rows to be picked.

A `Picker` class is defined in `picker.py`:
  1. `picking_process`: This process first reports the arrival of picker to the farm. If a row is allocated to the picker, this process first moves the picker to the start of the assigned row from current position. Picking involves moving forward along the nodes in the row and then returning to the start of the node (picking on the other side). When the current tray is full, moves it to his cart (`max_n_trays` > 1) and when `n_trays` reach `max_n_trays` initiate `transport_process` to unload trays to local storage.
  2. `transport_process`: This may involve one navigation to local storage (final unloading) or two navigations (from a row node to local storage and return).
  3. When there are no more rows to be allocated in the farm, the picker dumps all berries at local storage.
  4. Each picker publishes a `/<picker_name>/pose` topic when its position is updated (reaching a node).

# Known Issues:
  1. `env.step` is used to step through the des. 
     - some delay between the two clocks at the start. SimPy clock does not progress until the first `env.step`, but the all required process should be initialised before that.
     - Some delay (1-2 ms in ros/real-time clock) is observed between multiple events scheduled at the same simulation time.
     - However, chances of having multiple events scheduled at the same instance will be very small except at the start.
     - This delay does not seem to build up over time.

# TODO:
  1. The topological map should be replaced with the `topological_navigation` from `strands_project/strands_navigation` to simulate robot processes. 
  2. Although a topological map is created and is available in the mongodb, it still uses the Node and Graph classes for the farm and picker simulations. 
  3. More complex map may be defined in future using this. For example, long rows with two head lanes at both ends. A row may be assigned to two pickers who will start from either end, with associated local storage on the head lane on that side.
  4. The fork-map generation should be done outside the `Farm` class and should only be read from a yaml file / mongodb.
  5. Simulating robot process and integrate with the robots in gazebo
  6. Simulating ros-compatible picker processes and integrate with people movements in gazebo
  7. A definition of a scheduling interface (maybe inspired by strands, ROS msgs, and ROS action definitions to

    request tasks
    cancel tasks
    monitor task execution

  8. A fake scheduler that accepts tasks and hard-assigns them to a robot

