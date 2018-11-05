^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2018-11-02)
------------------
* Merge pull request `#155 <https://github.com/LCAS/RASberry/issues/155>`_ from gpdas/uv_updates
  Merging this. riseholme topo_maps updated
* Merge branch 'master' into uv_updates
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#153 <https://github.com/LCAS/RASberry/issues/153>`_ from adambinch/master
  Merging this. Added install targets for the new scripts in rasberry_navigation for topo_map updates
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge branch 'master' of github.com:LCAS/RASberry into uv_updates
* Corrected another error here
* Added install targets for the new scripts (to CMakeLists.txt), corrected a couple of errors and got the scripts to print some output to the terminal.
* Added a new directory `rasberry_navigation/scripts` containing the python scripts used to generate the latest topological maps. (`#151 <https://github.com/LCAS/RASberry/issues/151>`_)
  There is a short description in each script outlining what they do. Some are for general usage, others are for specific tasks.
  Also included are several text files specifying the positions of the polytunnel poles and the postions and way point numbers
  of the topological nodes.
* Updating the tmaps
  1. minor modifications in riseholme and riseholme_sim (e.g. orientation in some nodes, node-list reordered, map name corrected etc)
  2. riseholme-uv and riseholme-uv_sim are modified with @adambinch's new nodes for tunnel-b (only end nodes of four rows are kept and directions as in tunnel-a)
* Added a new directory `rasberry_navigation/scripts` containing the python scripts used to generate the latest topological maps.
  There is a short description in each script outlining what they do. Some are for general usage, others are for specific tasks.
  Also included are several text files specifying the positions of the polytunnel poles and the postions and way point numbers
  of the topological nodes.
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#147 <https://github.com/LCAS/RASberry/issues/147>`_ from adambinch/master
  Merging this. added nodes at bottom of second polytunnel.
* Second polytunnel (polytunnel B) added to topological maps. Python scripts that generated these maps to follow.
* Merge remote-tracking branch 'upstream/master'
* Merge pull request `#149 <https://github.com/LCAS/RASberry/issues/149>`_ from gpdas/master
  Merging this. Package for autonomous UV treatment - initial commits
* Merge branch 'master' into uv
* For topomap for simulation, added nodes at bottom of second polytunnel. Nodes at bottom of first polytunnel now have bi-directional edges.
* added nodes at bottom of second polytunnel. Nodes at bottom of first polytunnel now have bi-directional edges.
* Merge branch 'master' into uv
* Merge pull request `#146 <https://github.com/LCAS/RASberry/issues/146>`_ from gpdas/master
  Merging this. Sensor xacro updates and cleaning up old files
* Cleanup
* uv_treatment simulation cleanup
  1. costmap - inflation radius reduced to 0.5
  2. topo_map for riseholme-uv_sim added
  3. scenario sim_riseholme-uv.sh updated with new TMAP and MAP values
  4. cleanup in uv_treatment.py
* clean up and topo_map updates in uv_treatment
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* Merge branch 'master' of github.com:LCAS/RASberry into uv
* Merge pull request `#145 <https://github.com/LCAS/RASberry/issues/145>`_ from adambinch/master
  Merging this. costmap parameter modified and topo_map adjustment.
* Topo_map modified with exit node orientations straight
* Adjusted some of the costmap and local planner params (these were confirmed to work during a practical session with the robot at Riseholme.)
  Shifted end row nodes in topo map by 4cm in y direction. Adjusted position of node 10 in topo map for simulation.
* Initial commit of rasberry_uv
* Merge pull request `#144 <https://github.com/LCAS/RASberry/issues/144>`_ from adambinch/master
  Merging this. Nodes in topo map have been re-centred (they are now more accurately centred). The topo map for the sim has been updated to match the topo map.
* Nodes in topo map have been re-centred (they are now more accurately centred).
  The topo map for the sim has been updated to match the topo map.
  Gazebo food handling unit doorway has been widened to match the real building.
  Map and no go map for sim have been updated accordingly.
  RTF of sim has been increased by 10% by including the polytunnel arches as one mesh rather than generating each arch singularly.
  This mesh is specific for the riseholme environment. Therefore the inclusion of it has been made optional in the config file `rasberry_gazebo/config/gazebo/models_AB.yaml`
  (i.e. the arches can still be generated singularly to preserve the ability to generate polytunnels of arbitrary lengths).
* Merge pull request `#142 <https://github.com/LCAS/RASberry/issues/142>`_ from gpdas/master
  Merging this. Topological map for uv-rig navigation in riseholme
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge branch 'master' into master
* Merge pull request `#141 <https://github.com/LCAS/RASberry/issues/141>`_ from adambinch/master
  Merging this. Reduced influence zones of nodes inside rows such that they no longer…
* Topological map for uv-rig navigation in riseholme
* Reduced influence zones of nodes inside rows such that they no longer overlap
* Merge pull request `#139 <https://github.com/LCAS/RASberry/issues/139>`_ from adambinch/master
  Merging this. End lane nodes shifted back by half meter in topo map
* End lane nodes shifted back by half meter in topo map
* Merge pull request `#136 <https://github.com/LCAS/RASberry/issues/136>`_ from adambinch/master
  merging this. Adjusted positions of some nodes in topo map. Cleaned up some regions…
* Adjusted positions of some nodes in topo map. Cleaned up some regions in maps.
* Merge pull request `#135 <https://github.com/LCAS/RASberry/issues/135>`_ from adambinch/master
  Merging this. New nodes added to topological map located at charging points in food…
* New nodes added to topological map located at charging points in food handling unit.
  Influence zone of node (waypoint) 68 has been decreased.
  Corresponding changes made to the topological map for the simulation.
  Precision for roll value of hokuyo placement in `/rasberry_bringup/urdf/sensors.xacro` has been increased.
  Templates for the thorvald_006 have been added. Note that these are just a copy of the ones for thorvald_007 at the moment.
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge pull request `#133 <https://github.com/LCAS/RASberry/issues/133>`_ from adambinch/master
  Merging this. Adjustment of influence zone of the node outside FHU for smoother turning - both riseholme and riseholme_sim tmaps.
* Updated node for topo map
* Adjustment to one of the nodes in the topological map. Updated the rviz config so that the planner paths are the correct.
* Merge pull request `#132 <https://github.com/LCAS/RASberry/issues/132>`_ from adambinch/master
  Merging this. Hokuyo orientation modified. Map, no go map and topological map added for the new simulation environment.
* Map, no go map and topological map added for the new simulation environment.
  The plant models have been removed (for now) from this world to speed the sim up.
  I will look at ways of increasing the sim's real-time factor that will allow me to put them back in.
  `rasberry_bringup/launch/hokuyo.launch` and `rasberry_bringup/urdf/robot_007_sensors.xacro` have been updated
  due to the adjustment of the yaw of the hokuyo on the physical robot (it is now at zero degrees).
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
* New nodes added to the topological map permitting movement to and from the food handling unit.
  This new tmap file `rasberry_navigation/maps/riseholme.tmap` replaces the old file of the same name.
  The old tmap file has been renamed as `rasberry_navigation/maps/riseholme_old.tmap`.
* New Topological Map (`#126 <https://github.com/LCAS/RASberry/issues/126>`_)
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
* New topological map with nodes centred between the poles.
* Fixed an issue where the tf transform between odom and base link would be published twice when using the simulation. (`#125 <https://github.com/LCAS/RASberry/issues/125>`_)
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
* Fixed an issue where the tf transform between odom and base link would be published twice when using the simulation, once by gazebo and once by the ekf localisation node (see ). An environmen variable  has been added to  which can be set to false in a scenario.sh file if the user wants to use the simulation. The scenario  has been updated accordingly. The angular range of the laser scanner in  has been changed to span 180 degrees in front of the robot (as it was at the demo). Prior to this change the laser was hitting the robot's body.
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
* Merge branch 'master' of https://github.com/LCAS/RASberry
* WIP: riseholme development (`#121 <https://github.com/LCAS/RASberry/issues/121>`_)
  * added riseholme maps and uk robot 007 config files
  * .yaml and .tmap for riseholme topological map
  * more univeraal launch files
  * added scenario and more flexible tmule script
  * no sleeps necessary with new tmule
  * rise.pgm added
  * with topological nodes now
  * rise.pgm added
  * updated sensor measurements for switch to 008 frame
  * Added a script to start mapping
  * Added running IMU in start mapping script
  * Made start mapping script executable
  * tmap with charging, storage and base station
  * Cropping riseholme 2d map
  1. riseholme 2d map is cropped. Use riseholme.yaml with map_server.
  2. modified existing riseholme.tmap to riseholme_sim.tmap for the topological map for gazebo simulation
  3. added riseholme.tmap (a modified version of rise.tmap) to be used with riseholme.yaml 2D map.
  4. start_sim_risholme_unified.sh is updated to use riseholme_sim pointset from mongodb
  The above changed will make rise.yaml, rise.pgm, and rise.tmap deprecated. They are retained in this commit.
  * Robot objects in coordination now subscribes to robot_pose topics
  1. robot_pose topic is subscribed by the robot objects
  2. robot_pose is also used now for localising the robot in the topological map. this is to avoid low updates of the latched topic (closest_node) when used with rosbridge and rosduct interface.
  3. threading.Lock object is used to do the closest_node updates from two callbacks thread safe
  4. package and CMakeList are updated with rasberry_people_perception as additional dependency (for topological_localiser)
  * Removing the country prefix to robot config and sensor frame xacro
  * riseholme config fixes
  * Updated 2D map with FH building
  robot_007_sensors.xacro gazebo-hokuyo sensing range is modified to -135 to 85 degrees
  * adopted for server
  * SCENARIO_NAME is used for coordination config file in tmule config
  map_config_riseholme.yaml modified with nodes from the latest riseholme tmap.
  * load local config
  * fix for idle_robots without have topo_nav ready robots
  if a robot_name is in the map_config_scenario.yaml file, it was assumed to be idle. If the physical/simulated robot was not initialised its closest_node would be "none" and therefore wouldn't be considered while closest_robot to a task. now such robots wont be added to idle_robots and therefore tasks won't be removed from Queue unnecessarily.
  * New riseholme no_go map (with FH unit building)
  * local changes
  * added marvel and no go
  * shifted the map globally by 10cm!
  * some simple nav parameters that worked better, not great though
  * self.pose not to be updated from _update_closest_cb
  * added cache for topological_map (if server becomes unavailable)
  * fix for picker_node being None in add_task
  1. The actual fix is in picker_state_monitor, where the picker nodes are
  intialised as "none" now rather than None
  2. picker_marvel_localiser now checks for "none" rather than None before
  publishing closest and current nodes
  3. coordinator and robot in rasberry_coordination need values for
  static_nodes (storage etc) of the map.
  * fix: Task not added if picker is not localised
  1. If a picker who is not localised in the topomap is calling a robot, it is ignored and the callarobot state of the picker is reset to INIT
  2. fix in threading.Lock() usage for closest_node of robots
  3. fix for self._topo_nav.get_result() giving None
  * don't run picker localisation in robot
  * persistent topics updated
  * laser position centred
  * added .rasberryrc example file
  * angles shifted for center laser
  * params from riseholme demo
* Merge branch 'riseholme' of github.com:adambinch/RASberry
* Added a node `rasberry_gazebo/scripts/gazebo_people_tracker.py` that broadcasts info re the actors (pose, distance from robot etc) on to the `/people_tracker/positions` topic.
  This will permit the testing of the human aware navigation node (from the `strands_hri` repo) in simulation.
  Also added a launch file `rasberry_navigation/launch/human_aware_navigation.launch` to launch the human aware navigation node (testing of the node still to do).
  Number of actor laser beams have been increased from 36 to 180. This was to address an issue where actors were not decting obstacles 'until tthe last minute'.
* added riseholme maps and uk robot 007 config files
* Merge pull request `#120 <https://github.com/LCAS/RASberry/issues/120>`_ from gpdas/master
  Merging this. Post Noway workshop demo Cleanup
* CMakeLists.txt and package.xml updated with dependencies and install targets
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge branch 'master' into master
* Merge pull request `#119 <https://github.com/LCAS/RASberry/issues/119>`_ from tuandle/master
  Selectively choose config file for localization
* fix group tag
* Merge branch 'master' of github.com:LCAS/RASberry
* selectively choose config file for localization base on usage of imu
* Merge pull request `#115 <https://github.com/LCAS/RASberry/issues/115>`_ from tuandle/master
  WIP properly fuse IMU measurements (including orientation, gyro and acceleration) for localization
* properly fuse imu measurements
* Merge pull request `#2 <https://github.com/LCAS/RASberry/issues/2>`_ from LCAS/master
  Sync latest version of demo
* Merge pull request `#114 <https://github.com/LCAS/RASberry/issues/114>`_ from gpdas/master
  merging this. fixes to imu and marvel to topo localisation
* imu0: imu/data in norway_imu_ekf.yaml
* Merge branch 'master' of github.com:LCAS/RASberry
* readded amcl
* wip to get robot up and running (`#112 <https://github.com/LCAS/RASberry/issues/112>`_)
  * tmux shell script updates
  * IMU EKF params correction. teleoperation enabled with robot launch
* Merge branch 'master' into master
* IMU EKF params correction. teleoperation enabled with robot launch
* Rasberry Topological Navigation testcase (`#101 <https://github.com/LCAS/RASberry/issues/101>`_)
  * Rasberry Topological Navigation testcase
  * Rasberry Topological Navigation testcase
* Merge branch 'master' of https://github.com/LCAS/RASberry
* wip
* Merge pull request `#111 <https://github.com/LCAS/RASberry/issues/111>`_ from gpdas/master
  merging this. tweaks in the tmule configs
* cropped map
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
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#97 <https://github.com/LCAS/RASberry/issues/97>`_ Now using seperate map_server.launch with no_go_map
  Merging this.
  Cleaningup of launch files - this pr separates `amcl` and `map_server` from `move_base` launch files. A separate launch file is already there for `amcl` and a map_server launch file (modified in this PR) in rasberry_move_base.
* Merge pull request `#1 <https://github.com/LCAS/RASberry/issues/1>`_ from gpdas/pr97
  @YiannisMenex merging some additional changes in the movebase launch files
* further changes in movebase launch files
  1. amcl, map_server are no longer launched from any of the movebase launch files
  2. norway_topo_nav.launch is removed - should be replaced with a tmule config in future
* Now using seperate map_server.launch with no_go_map
  -Removed map_server from move_base_dwa.launch (+ the needed arguments)
  -Editted the map_server.launch to also include the no_go_map
  -Added the no_go_map file (pgm + yaml) in rasberry_gazebo/maps
  -New layer "no_go_layer" in costmap_common_params.yaml
  -Added the no_go_map layer ("no_go_layer") in global_costmap
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
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#84 <https://github.com/LCAS/RASberry/issues/84>`_ from ThomasDegallaix/master
  First commit of topoNav testcases
* Rasberry Navigations testcase
* Topological navigation testcase
* Testcase update
* nw
* new_work
* First commit of topoNav testcases
* First commit for the topological navigation testcases
* w.i.p.
* correct email
* Contributors: Adam Binch, Alexander Gabriel, Gautham P Das, Jaime Pulido Fentanes, Johnmenex, LCASABU02, Marc Hanheide, ThomasDegallaix, Tuan Le, Yiannis Menexes, adambinch, gpdas, jailander, thorvald, tuandle

0.0.4 (2018-07-18)
------------------

0.0.3 (2018-07-16)
------------------

0.0.2 (2018-05-21)
------------------
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge branch 'master' of https://github.com/LCAS/RASberry into topo_nav
* Merge pull request `#30 <https://github.com/LCAS/RASberry/issues/30>`_ from Jailander/master
  Removing unnecessary files and adding launch file for GPSs
* Merge pull request `#29 <https://github.com/LCAS/RASberry/issues/29>`_ from tuandle/master
  merge rasberry_navigation
* removing unnecessary files
* dual_ekf_navsat_2gps.launch output: odometry from r_l fusing onlu imu and robot's odom;output from navsat_transform_node includes original gps measurements and filtered measurements
* first tried to setup 2 gps for r_l
* Adding param templates and template launch files for localisation filters
* Contributors: Jaime Pulido Fentanes, eirikgarsol, gpdas, tuandle

0.0.1 (2018-03-05)
------------------
* navigation added
* Contributors: Marc Hanheide
