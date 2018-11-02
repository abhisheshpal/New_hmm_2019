^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2018-11-02)
------------------
* Merge pull request `#144 <https://github.com/LCAS/RASberry/issues/144>`_ from adambinch/master
  Merging this. Nodes in topo map have been re-centred (they are now more accurately centred). The topo map for the sim has been updated to match the topo map.
* Nodes in topo map have been re-centred (they are now more accurately centred).
  The topo map for the sim has been updated to match the topo map.
  Gazebo food handling unit doorway has been widened to match the real building.
  Map and no go map for sim have been updated accordingly.
  RTF of sim has been increased by 10% by including the polytunnel arches as one mesh rather than generating each arch singularly.
  This mesh is specific for the riseholme environment. Therefore the inclusion of it has been made optional in the config file `rasberry_gazebo/config/gazebo/models_AB.yaml`
  (i.e. the arches can still be generated singularly to preserve the ability to generate polytunnels of arbitrary lengths).
* Merge pull request `#132 <https://github.com/LCAS/RASberry/issues/132>`_ from adambinch/master
  Merging this. Hokuyo orientation modified. Map, no go map and topological map added for the new simulation environment.
* Map, no go map and topological map added for the new simulation environment.
  The plant models have been removed (for now) from this world to speed the sim up.
  I will look at ways of increasing the sim's real-time factor that will allow me to put them back in.
  `rasberry_bringup/launch/hokuyo.launch` and `rasberry_bringup/urdf/robot_007_sensors.xacro` have been updated
  due to the adjustment of the yaw of the hokuyo on the physical robot (it is now at zero degrees).
* Merge remote-tracking branch 'upstream/master'
* set model path automatically from env hook (`#130 <https://github.com/LCAS/RASberry/issues/130>`_)
* New nodes added to the topological map permitting movement to and from the food handling unit. (`#128 <https://github.com/LCAS/RASberry/issues/128>`_)
  * added riseholme maps and uk robot 007 config files
  * more univeraal launch files
  * added scenario and more flexible tmule script
  * no sleeps necessary with new tmule
  * Added a node `rasberry_gazebo/scripts/gazebo_people_tracker.py` that broadcasts info re the actors (pose, distance from robot etc) on to the `/people_tracker/positions` topic.
  This will permit the testing of the human aware navigation node (from the `strands_hri` repo) in simulation.
  Also added a launch file `rasberry_navigation/launch/human_aware_navigation.launch` to launch the human aware navigation node (testing of the node still to do).
  Number of actor laser beams have been increased from 36 to 180. This was to address an issue where actors were not decting obstacles 'until tthe last minute'.
  * removed outdated files
  * changes allow the sim to be run using the tmule set up
  * Fixed an issue where the tf transform between odom and base link would be published twice when using the simulation, once by gazebo and once by the ekf localisation node (see ). An environmen variable  has been added to  which can be set to false in a scenario.sh file if the user wants to use the simulation. The scenario  has been updated accordingly. The angular range of the laser scanner in  has been changed to span 180 degrees in front of the robot (as it was at the demo). Prior to this change the laser was hitting the robot's body.
  * New topological map with nodes centred between the poles.
  * New nodes added to the topological map permitting movement to and from the food handling unit.
  This new tmap file `rasberry_navigation/maps/riseholme.tmap` replaces the old file of the same name.
  The old tmap file has been renamed as `rasberry_navigation/maps/riseholme_old.tmap`.
  * New Gazebo world (`rasberry_gazebo/worlds/riseholme_new.world`) with improved polytunnels and surrounding environment.
  To replace `rasberry_gazebo/worlds/riseholme.world` once it has been mapped.
  The many changes made to the `rasberry_gazebo` package were made so that this new simulation environment could be produced.
  Removed some unnecessary files. The gazebo world `real_map.world` now just contains the polytunnel poles.
* New Gazebo world (`rasberry_gazebo/worlds/riseholme_new.world`) with improved polytunnels and surrounding environment.
  To replace `rasberry_gazebo/worlds/riseholme.world` once it has been mapped.
  The many changes made to the `rasberry_gazebo` package were made so that this new simulation environment could be produced.
  Removed some unnecessary files. The gazebo world `real_map.world` now just contains the polytunnel poles.
* Merge remote-tracking branch 'upstream/master'
* Added  `gazebo_people_tracker.py` and tmule config for simulation (`#122 <https://github.com/LCAS/RASberry/issues/122>`_)
  * added riseholme maps and uk robot 007 config files
  * more univeraal launch files
  * added scenario and more flexible tmule script
  * no sleeps necessary with new tmule
  * Added a node `rasberry_gazebo/scripts/gazebo_people_tracker.py` that broadcasts info re the actors (pose, distance from robot etc) on to the `/people_tracker/positions` topic.
  This will permit the testing of the human aware navigation node (from the `strands_hri` repo) in simulation.
  Also added a launch file `rasberry_navigation/launch/human_aware_navigation.launch` to launch the human aware navigation node (testing of the node still to do).
  Number of actor laser beams have been increased from 36 to 180. This was to address an issue where actors were not decting obstacles 'until tthe last minute'.
  * removed outdated files
  * changes allow the sim to be run using the tmule set up
* Merge branch 'master' into master
* added real world map (`#124 <https://github.com/LCAS/RASberry/issues/124>`_)
* Added a node `rasberry_gazebo/scripts/gazebo_people_tracker.py` that broadcasts info re the actors (pose, distance from robot etc) on to the `/people_tracker/positions` topic.
  This will permit the testing of the human aware navigation node (from the `strands_hri` repo) in simulation.
  Also added a launch file `rasberry_navigation/launch/human_aware_navigation.launch` to launch the human aware navigation node (testing of the node still to do).
  Number of actor laser beams have been increased from 36 to 180. This was to address an issue where actors were not decting obstacles 'until tthe last minute'.
* Merge pull request `#120 <https://github.com/LCAS/RASberry/issues/120>`_ from gpdas/master
  Merging this. Post Noway workshop demo Cleanup
* CMakeLists.txt and package.xml updated with dependencies and install targets
* removed `maps` from rasberry-gazebo install targets
* Post Noway workshop demo Cleanup
  1. deleted:    rasberry_bringup/launch/rasberry_simulation.launch. Use robot_brinup.launch.
  2. multi-robot simulations: This is not our current focus, however keeping these updated with other developments.
  - deleted:    rasberry_bringup/launch/rasberry_simulation_multi.launch. Use robot_bringup_multisim.launch (new file)
  - new file:   rasberry_move_base/launch/move_base_dwa_multisim.launch
  - renamed:    rasberry_move_base/launch/move_base_teb_multi.launch -> rasberry_move_base/launch/move_base_teb_multisim.launch
  - renamed:    rasberry_bringup/scripts/start_sim_multi.sh -> rasberry_bringup/scripts/start_multisim.sh
  3. all maps are now removed from rasberry_gazebo. already moved all maps to rasberry_navigation
  - modified:   rasberry_bringup/launch/rasberry_simulation_velo_k2.launch
  - modified:   rasberry_gazebo/launch/actor_move_base_dwa.launch
  - deleted:    rasberry_gazebo/maps/*
  - modified:   rasberry_move_base/launch/map_server.launch
  - modified:   rasberry_people_perception/launch/ultrasonic_localisation.launch
  4. deleted:    rasberry_bringup/launch/thorvald_realtime.launch. Use robot_brinup.launch.
  5. new file:   rasberry_bringup/urdf/norway_robot_003_sim_sensors.xacro.
  - Modified of the xacro on thorvald_003 for norway_poles demo.
  - In simulation, the coordiate frames of hokuyo is not right (some rays are hitting the robot itself with 180 degree view)
  6. shell script updates (before tmule configs to test things, these are used).
  - modified:   rasberry_bringup/scripts/start_sim.sh
  - renamed:    rasberry_bringup/scripts/start_sim_multi.sh -> rasberry_bringup/scripts/start_multisim.sh (multi-robot simulation. only upto move_base)
  - modified:   rasberry_bringup/scripts/start_sim_norway_poles_unified.sh. This can be used to simulate norway_poles scenario with one robot and multiple pickers from a single roscore
  - modified:   rasberry_bringup/scripts/start_sim_riseholme_unified.sh. This can be used to simulate riseholme_sim scenario with one robot and multiple pickers from a single roscore
  7. modified:   rasberry_bringup/tmule/norway_poles_sim.yaml.
  - changes in tmule configs from robot.
  - ROBOT_NAME by truncating "-" from hostname, mongodb directory and launch, and reduced sleep delays.
  - not tested, but could be modified in future for simulating the scenarios with multi-roscore as in real cases with rosduct. Keeping updated along with other changes.
  8. modified:   rasberry_move_base/launch/move_base_dwa.launch. remapping odom to odometry/base_raw
  9. coorindation/scheduling related:
  - new file:   rasberry_coordination/config/map_config_riseholme_sim.yaml. Configuration file to be used with simple_task_executor_node for riseholme_sim scenario.
  - modified:   rasberry_coordination/scripts/simple_task_executor_node.py. Now passes "unified" parameter to PickerStateMonitor as well.
  - modified:   rasberry_coordination/src/rasberry_coordination/coordinator.py.
  - when "unified" is true (single roscore) only one robot will be added due to the base namespacing of many topological navigation topics.
  - now checks for start and goal nodes being "none"
  - now checks for route is None while getting route to picker (to find the nearest robot). None could come if there is no possible path.
  - minor rosinfo msg updates
  - modified:   rasberry_coordination/src/rasberry_coordination/picker_state_monitor.py
  - now takes "unified" status and when it is true, assumes there is only one robot
  - modified:   rasberry_coordination/src/rasberry_coordination/robot.py
  - minor rosinfo msg updates
  - fixed some bugs in checking topo_nav action goal status.
  - Known issues (to be investigated later):
  - collectTray goal is cancelled, if any topo_nav action goal underneath is aborted or recalled by the action server. It is still not elegant way of doing it, as there could be better feedback.
  - if there is only one idle robot and a path does not exist from the robot to the picker, the collecttray task is still assigned to that robot. this should be avoided.
  - with riseholme.tmap, some nodes could be used in rviz to set top_nav goals, same nodes when used to find a path, failed as well as those tasks were aborted/recalled (?)
* Merge pull request `#1 <https://github.com/LCAS/RASberry/issues/1>`_ from LCAS/master
  sync with LCAS
* WIP: initial framework for task coordination (`#69 <https://github.com/LCAS/RASberry/issues/69>`_)
  * initial framework for task coordination
  * rosduct setuo
  * better namespacing
  * added install target
  * robot_pose
  * delete obsolete file
  * Initial crude implementation of robot assignment for tray collection
  1. AddTask add a Task with start_node_id=picker_node
  2. A Robot class with
  - CollectTray.action. No proper feedback or result at this stage
  - tray_loaded and tray_unloaded services - to trigger next stage of CollectTrayAction
  3. Some basic framework for picker localisation in topo_map from picker_pose - may not be needed if picker_localisation is based on topo_map
  * Adding missing action files
  * rosduct setup for toponav and actions
  * moved callarobot
  * working rosduct ready for testing
  * added topo localisation
  * update callarobot
  * Updates to CollectTray action service
  CollectTray feedback has been modified to include route info which would be available from topological_navigation action
  * Initial implemetation of Picker state monitor and publisher
  Picker states are received and picker states are stored
  New state setting not working at this stage, but basic framework is there.
  * minor
  * cleaning up picker state monitor
  picker's states are received. when state is reset from web interface, the received msg is also used to reset internal state of a picker
  a picker's state can be set using set_state
  all pickers are assumed to be in the format "pickerxy" and the people_perception node would be giving picker positions in /pickerxx/posestamped etc.
  * Coorindator updates:
  1. picker_state_monitor now can monitor and set states in CAR
  2. task assignmment - closest robot is found now
  Pending:
  1. task assignment from picker_state_monitor
  2. task cancellation if picker cancels the task
  3. picker state changes based on robot progress
  * Scheduler - Mostly there, still some bugs to be fixed
  Flow:
  1. picker_state_monitor listens to picker states and localisation topics.
  2. add_task in coordinator is called when the picker calls a robot (CAR)
  3. coordinator selects each task, if idle_robots available. selet the closest robot
  4. collect_tray action in the robot is called
  5. robot goes to picker location. waits for loading. once loaded goes to storage. waits for unloading.
  6. robot sends collect_tray feedbacks, which are modified by coordinator as task updates to picker_state_monitor.
  7. picker_state_monitor sets picker states in CAR using these feedbacks
  Known issues:
  1. When the picker_state_monitor is initialised, if any picker state was not INIT and was reset through CAR web interface, tasks are not added for any pickers
  2. robot does not seem to wait for loading state change
  Other important updates:
  1. start_sim.sh updated for launching different necessary components for the coordination simulation
  2. topological_navigation.launch from topological_navigation package has been split into two launch files - map_manager_central and topological_navigation_robot. they will be launched at different places.
  3. picker topic name spaces corrected in people_perception
  4. topological_localiser had a wrong class name, which is fixed now.
  5. ultrasonic_localisation.launch in people_perception is modified with arguments
  6. New service srv/CancelTask.srv
  7. new message msg/TaskUpdates.msg
  8. action/CollectTray.action modified with task_id field in both feedback and result for meaningful feedback to picker_monitor
  9. CMakeLists.txt is updated with msg and srv components
  * Fixes and more fixes. Mostly working except cancellation between task execution.
  1. State ACCEPTED corrected to ACCEPT
  2. picker_prev_state updates were not proper. fixed
  3. a hack to get things work with the name sapces. topological navigation related topics/services/actions are in the root namespace while all other robot related things in rasberry_coordination are in /robot_id/ namespace. everything sho$
  4. fixed issues in tray_loaded_cb and tray_unloaded_cb
  5. feedback fixed in wait_for_laoding and wait_for_unloading
  6. topological navigation result was not properly read from go_to_picker and go_to_storage
  7. tray loaded status from robot after maxed out load delay is now used for picker state change
  8. now avoids multiple tasks when new car_events arrive with same now:called prev:init
  known issues:
  1. cancellation of tasks
  * Task addition, waiting for robot to be free, picker state changes with action progress/CAR updates are working.
  Another fixed node base_station added. collect_tray action involves the following topo_nav actions to_picker -> to_storage -> to_base_station
  TODO: Collect_Tray action cancellation
  * Fixed waypoints (base station, storage and charging) for norway_poles added
  * Cleaning up by adding specific simulation launch script
  1. config files for coordination/sample_task_executor node
  2. poles world file renamed to norway_poles from norwayPolytunnel
  3. tmap for the unidirectional rows and other static nodes is added - norway_poles.tmap / pointset: norway_poles
  4. rviz configuration with two picker poses
  5. single shell script to load tmux windows for all necessary modules
  * Bash shell scripts to start tmux sessions with coorindation running in server
  1. Shell scripts in rasberry_bringup/scripts - run as rosrun rasberry_bringup start_sim_norway_poles_xxx.sh
  2. rosduct launch files for defining connection to rosbridge in coordination server. robot_websocket_adapter.launch which in turn includes the other robot_websocket_xxx.launch files
  * Rosduct websocket launch files for running at the coordination server side
  examples usage is already in the rasberry_bringup/start_sim_norway_poles_server.sh
  * Updating maps in navigation from gazebo
  maps directory in _gazebo to be removed later and will be replaced by maps dir in _navigation
  * tmux scripts for robots updated with rasberry_localisation.launch and simple_sim=false
* Wip tmule (`#98 <https://github.com/LCAS/RASberry/issues/98>`_)
  * WIP: tmule for norway_poles demo
  1. tmule config files for launching actual robot and simulation - norway_poles and norway_poles_sim
  2. parameterised ability launch files (these needs work):
  - rasberry_bringup/robot_bringup.launch
  - rasberry_navigation/localisation.launch
  - rasberry_move_base/map_server.launch (will be coming separately from PR97)
  - rasberrymove_base/move_base_dwa.launch (this needs some rework and should come from PR97 soon)
  3. map files moved from rasberry_gazebo to rasberry_navigation
  * tmule config file changes
  1. config files updated with delays - simulation configuration (norway_poles_sim) tested and launching all windows.
  2. robot_bringup.launch now launches the gazebo world as well
  3. norway_poles is the name to be used with all files related the demo
  known issues:
  1.  high delay 10-20s in all terminals. this could be fine tuned later
  2.  even after sending terminate, some of ros processes were not killed
* Fixed error causing the robots laser scan data from getting into the actor's local costmap. (`#102 <https://github.com/LCAS/RASberry/issues/102>`_)
  Some tuning of the teb local planner for the actors - actors now avoid each other and the robot and can navigate the rows smoothly.
  Laser model samples have been reduced from 720 to 36 - no issues with performance.
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#97 <https://github.com/LCAS/RASberry/issues/97>`_ Now using seperate map_server.launch with no_go_map
  Merging this.
  Cleaningup of launch files - this pr separates `amcl` and `map_server` from `move_base` launch files. A separate launch file is already there for `amcl` and a map_server launch file (modified in this PR) in rasberry_move_base.
* Now using seperate map_server.launch with no_go_map
  -Removed map_server from move_base_dwa.launch (+ the needed arguments)
  -Editted the map_server.launch to also include the no_go_map
  -Added the no_go_map file (pgm + yaml) in rasberry_gazebo/maps
  -New layer "no_go_layer" in costmap_common_params.yaml
  -Added the no_go_map layer ("no_go_layer") in global_costmap
* Merge branch 'master' of github.com:LCAS/RASberry
* added installation instructions to rasberry_gazebo readme (`#95 <https://github.com/LCAS/RASberry/issues/95>`_)
* Merge branch 'master' of github.com:LCAS/RASberry
* Norway topo-nav for simulation (`#89 <https://github.com/LCAS/RASberry/issues/89>`_)
  * Norway topo-nav for simulation
  The launch file that launches everything is the rasberry_navigation/launch/norway_topological_navigation.launch.
  MongoDB must be launched before launching this file, using ''rosparam set use_sim_time true''.
  Norway simulation files for topo-nav also created (amcl, costmap, move_base, norway_world.launch, new map/tmap/yaml files)
  * Removed mongoDB
  * Exposed params on existing launch files for topo-nav
  -Created new launch file that launches the topological navigation, with arguments "db_path" and "topo_map".
  -Removed duplicate files and exposed some parameteres of the already existing launch files.
  *Launch files with exposed args:
  -rasberry_bringup rasberry_simulation.launch
  -rasberry_gazebo world.launch (switched world_name from "value" to "default")
  -rasberry_move_base amcl.launch
  -rasberry_move_base move_base_dwa.launch
* nw
* new_work
* First commit for topoNav testcases
* Merge pull request `#83 <https://github.com/LCAS/RASberry/issues/83>`_ from adambinch/master
  Corrected an error where actor laser data was failing to make it into…
* Corrected an error where actor laser data was failing to make it into the local cost map (actors now have dynamic obstacle avoidance).
  Actors no longer use amcl to localize - they now use their ground truth locations.
  To Do: actors avoid obstacles most of the time but tuning of the teb local planner needed. README.md needs updating.
* Merge pull request `#75 <https://github.com/LCAS/RASberry/issues/75>`_ from adambinch/master
  Error in rasberry_gazebo/urdf/actor.xacro has been corrected
* Error in rasberry_gazebo/urdf/actor.xacro has been corrected
* Merge pull request `#74 <https://github.com/LCAS/RASberry/issues/74>`_ from adambinch/master
  Implemented move_base nav for multiple actors
* w.i.p.
* w.i.p.
* The package rasberry_actors has been subsumed into rasberry_gazebo.
  Move base has been implemented for multiple actors using teb local planner.
  A node has been created for publishing sequences of nav goals - can be used for actors or robots (see rasberry_gazebo/scripts/move_base_seq.py).
  A launch file for use with the physical robot (no sim elements) only has been made. See rasberry_gazebo/launch/thorvald_realtime.launch.
  To fix: actors only avoiding obstacles in the no go map. Probably laser data not making it into local costmap.
* Changes to CMakeLists and package.xml
* Changes for testing (not to be merged)
* Contributors: Adam Binch, Gautham P Das, Johnmenex, Marc Hanheide, ThomasDegallaix, Tuan Le, Yiannis Menexes, adambinch, alexander-gabriel, gpdas

0.0.4 (2018-07-18)
------------------
* removed depend to gazebo_ros
* use gazebo 8
  closes `#65 <https://github.com/lcas/rasberry/issues/65>`_
* Added simulation with velodyne and kinect2, rviz config file to visualize them and changed actor to use gpu based simulated laser
* Merge branch 'master' of github.com:LCAS/RASberry
* Changing topological map name
* adding simulation start-up script
* Merge branch 'master' of github.com:LCAS/RASberry
  conflict resolved in rasberry_gazebo/worlds/thorvald_AB.world
* World file changes because of polytunnel definition interchange in models_AB.yaml
* Contributors: Jaime Pulido Fentanes, Marc Hanheide, gpdas, mfernandezcarmona@lincoln.ac.uk

0.0.3 (2018-07-16)
------------------
* Added more Local Planners -> check description
  *EBand Local Planner
  -Launch file: "move_base_eband.launch"
  -Parameters under directory "rasberry_move_base/config/eband/"
  *Teb Local Planner
  -Launch file: "move_base_teb.launch"
  -Parameters under directory "rasberry_move_base/config/teb/"
  *DWA Local Planner
  -Launch file: "move_base_dwa.launch"
  -Parameters under directory "rasberry_move_base/config/dwa/"
* Seperated the gazebo world with polytunnels and actor spawning into seperate packages (rasberry_gazebo and actor_gazebo, respectively).
  There is also a rasberry_bringup package that launches everything together
* Merge branch 'master' of https://github.com/jailander/RASberry
  # Conflicts:
  #	rasberry_gazebo/package.xml
* removing unnecessary stuff
* rasberry_move_base config files
* Merge branch 'master' of https://github.com/adambinch/RASberry
* Merge branch 'master' of https://github.com/adambinch/RASberry
* Added a new launch file 'gazebo_single_AB.launch'.
  This provides a switch to turn actors on or off and also launches
  the teleop_xbox nodes so that Thorvald can be controlled with the
  xbox controller. Note that actors are now specified in another
  new launch file 'include_actors.launch'
* adding rasberry_move_base_package
* gazebo dependency removed from cmakelists and package.xml
* Merge branch 'master' of https://github.com/adambinch/RASberry
* All packages found in cmakelists are now included in package.xml
* Merge branch 'master' of https://github.com/adambinch/RASberry
  # Conflicts:
  #	rasberry_gazebo/package.xml
* More changes for testing ...
* Merge branch 'master' of https://github.com/adambinch/RASberry
* Merge branch 'master' of https://github.com/LCAS/RASberry
  # Conflicts:
  #	rasberry_gazebo/launch/gazebo_single.launch
* Some changes for Jaime to check.
* The type 2 actor's laser scanner is now included in the tf tree.
  The launch file 'thorvald_world_AB.launch' now launches the Thorvald robot model into the gazebo world.
  The README.md has been adjusted to reflect these changes.
* making enclosure world default environment for now
* adding world and navigation maps
* Contributors: Jaime Pulido Fentanes, Johnmenex, adambinch

0.0.2 (2018-05-21)
------------------
* Merge pull request `#51 <https://github.com/LCAS/RASberry/issues/51>`_ from adambinch/master (RT Factor improvements and TF tree fixes for multi actors)
  Tested.
  - RT Factor improved to 0.87~0.97 from earlier 0.17 by combining multiple plant pots into one model and thereby reducing the number of plant models in simulation.
  - TF tree fixes for multi actors - Basic frame work ready for working on actor navigation
* Fixed the type-2 actor's tf trees. Thank you Gautham P Das!
* Further improvements to the computational efficiency of the sim.
  For the example polytunnel/actor configurations in this package the real-time factor should now be 0.97.
  Collision properties of the type-2 actor have been adjusted.
* The real-time factor of the simulation has been greatly increased - from 17% to 87%.
  The range of the type-2 actor's laser scanner has been increased - from 10m to 50m.
* Merge pull request `#49 <https://github.com/LCAS/RASberry/issues/49>`_ from adambinch/master (controllable humans)
  Merging this.
  Seems working with two human-robots with individual `cmd_vel` and `odom` topics, and an human actor walking along waypoints.
* Merge pull request `#50 <https://github.com/LCAS/RASberry/issues/50>`_ from gpdas/master (Secondary head lane & Config file format changes)
  Tested OK.
  No major change in any agent behaviours.
* Generating topological maps for rasberry_gazebo & secondary head lane support
  Generating topological maps for rasberry_gazebo:
  - Polytunnel configuration read from rasberry_gazebo/config/models_AB.yaml
  - Hard assumptions
  - polytunnels are stacked, but different  x_offsets are possible
  - the polytunnel with the lowest y coordinate offset is defined first
  - remaining polytunnels are defined in the order of their ascending y coordinate position
  - modified rasberry_gazebo/config/models_AB.yaml with the above assumption
  Config file modifications:
  - head_node_x is replaced with x_offset of first row_node and head_node position is calculated using this and head_row_node_dist
  - new config param second_head_lane to indicate a secondary head lane at the other end
  - new config_param map_name to indicate the topological map name to be used with the config file
  - due to a possible second head lane, head nodes in primary head lane are named as pri_hn_xx and those in secondary are sec_hn_xx. Similar name changes are made to edges of these nodes.
  - map files recreated for all config files and for the current gazebo config file
  Config file is no longer loaded into rosparam server.
  - config file need to be passed as an argument, wherever required
  - launch files used to load config params from config files are removed
  - a new launch file for broadcasting a map topic is added
  - config_utils.py updated to work with the above changes. only major change is to pass config_file_handler as in argument
  CMakeList.txt
  - added new script for map generation from gazebo config in install target
  config/
  - Additions: x_offset, x_offset_func, y_offset, second_head_lane
  - Deletions: head_nodex, head_node_x_func
  maps/
  - tmap and yaml files for rasberry_gazebo/config/models_AB.yaml
  scripts/
  - config_file name is needed as an argument for all existing scripts
  - changes corresponding to config_params in fork_map_generator.py
  config_utils.py
  - config_params are read from config_file, not anymore from paramserver
  - modifications to address config_param changes
  generate_map.py
  - config_param changes
  - secondary head lane support
  topo.py
  - head_nodes[row_id] is an array. second value appended only when there are secondary_head_lane
  - row_info[row_id] is now [primary_head_node, start_row_node, last_head_node, local_storage_node, secondary_head_node (if present)]
  - get_row_info() renamed as set_row_info()
  visualise.py
  - support for secondary head lane
* Some minor changes to the README.md and comments in other files.
* An issue with the actors generated using the config file `./config/actors_AB.yaml` is that they cannot be dynamically controlled during simulation time.
  Therefore another type of actor has been made available in this package that can be controlled at runtime. These are robots with a human mesh, controlled with the standard
  `libgazebo_ros_planar_move plugin`.
* Merge pull request `#45 <https://github.com/LCAS/RASberry/issues/45>`_ from adambinch/master
  List of python package dependencies added to README.md
* List of python package dependencies added to README.md
* Merge pull request `#44 <https://github.com/LCAS/RASberry/issues/44>`_ from adambinch/master
  Size of plant model mesh reduced (from 140kb to 30kb)
* Size of plant model mesh reduced (from 140kb to 30kb)
* Merge pull request `#41 <https://github.com/LCAS/RASberry/issues/41>`_ from adambinch/master
  adding world generator script
* On branch master
  Your branch is up-to-date with 'origin/master'.
  Changes to be committed:
  modified:   rasberry_gazebo/README.md
  modified:   rasberry_gazebo/config/actors_AB.yaml
  modified:   rasberry_gazebo/config/models_AB.yaml
  modified:   rasberry_gazebo/models/dummy_arch/model.config
  modified:   rasberry_gazebo/models/plant/model.sdf
  modified:   rasberry_gazebo/models/plant2/model.sdf
  deleted:    rasberry_gazebo/models/plant2/plant2.dae
  modified:   rasberry_gazebo/models/pole/model.sdf
  modified:   rasberry_gazebo/scripts/add_to_world.py
  modified:   rasberry_gazebo/scripts/generate_world.py
  modified:   rasberry_gazebo/worlds/thorvald_AB.world
  Untracked files:
  rasberry_gazebo/models/plant2/plant2.stl
  rasberry_gazebo/models/plant3/
* Many simulation improvements including movable model humans ('actors').
* adding world generator script
* Merge branch 'master' of https://github.com/LCAS/RASBerry
* Merge branch 'master' of https://github.com/LCAS/RASBerry into visualise_pickers
* Merge pull request `#37 <https://github.com/LCAS/RASberry/issues/37>`_ from Jailander/master
  adding arch to package
* adding archs to package
* Merge branch 'master' of https://github.com/LCAS/RASBerry into des_topo_nav
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge pull request `#35 <https://github.com/LCAS/RASberry/issues/35>`_ from Jailander/master
  adding polytunnels file
* adding polytunnels file
* Merge pull request `#34 <https://github.com/LCAS/RASberry/issues/34>`_ from Jailander/master
  adding first polytunnel simulation
* adding first polytunnel simulation
* Merge branch 'master' of https://github.com/LCAS/RASberry into des_topo_nav
* Merge pull request `#33 <https://github.com/LCAS/RASberry/issues/33>`_ from Jailander/master
  adding sensor frame and polytunnels world v0 to gazebo single
* adding sensor frame and polytunnels world v0 to gazebo single
* Merge branch 'master' of https://github.com/LCAS/RASberry into des_topo_nav
* created first simple gazebo launch file
* Merge branch 'master' of https://github.com/LCAS/RASberry into topo_nav
* Contributors: Gautham P Das, Jaime Pulido Fentanes, Marc Hanheide, adambinch, eirikgarsol, gpdas

0.0.1 (2018-03-05)
------------------
* skeleton for RASberry simulation
* Contributors: Marc Hanheide
