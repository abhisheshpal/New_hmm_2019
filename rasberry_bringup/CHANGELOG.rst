^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#156 <https://github.com/LCAS/RASberry/issues/156>`_ from gpdas/uv_updates
  Tested in simulation. Merging this.
* rasberry_uv updates + minor changes elsewhere
  1. sim_riseholme-uv.yaml tmule config is removed and riseholme-uv.yaml can be used for both the physical robot as well as simulation
  2. config file (rasberry_uv/config/riseholme.yaml) for configuring which rows have to be treated and start and end nodes of the rows
  3. rasberry_uv/uv_rig.py now takes an additional argument `use_sim` to control the serial connection based on whether it is a simulation or not
  4. rasberry_uv/src/uv_treatment.py no longer takes `use_sim` as an argument
  5. rasberry_uv/CMakeLists.txt install targets updated
  6. rasberry_bringup/robot_bringup.launch takes an new argument with_gui to enable/disable gazebo gui. tmule configs using this launch file have been updated.
* tidied up and made version compatible (`#152 <https://github.com/LCAS/RASberry/issues/152>`_)
  * tidied up and made version compatible
  * dep update for rasberry_bringup (persistent_topics)
* Merge pull request `#154 <https://github.com/LCAS/RASberry/issues/154>`_ from gpdas/master
  Merging this. Task cancellation & other updates in rasberry_coordination
* Merge remote-tracking branch 'upstream/master'
* Merge branch 'coord'
  CollectTray action is updated with task instead of task_id and picker_node
* Merge pull request `#149 <https://github.com/LCAS/RASberry/issues/149>`_ from gpdas/master
  Merging this. Package for autonomous UV treatment - initial commits
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* Merge branch 'master' into uv
* fixing the launch file
* Merge branch 'master' into uv
* Merge pull request `#146 <https://github.com/LCAS/RASberry/issues/146>`_ from gpdas/master
  Merging this. Sensor xacro updates and cleaning up old files
* Updated robot sensor xacros
  IMUs disabled
  ROSBRIDGE_IP set to 10.8.0.18
* rasberry_coordination CMakeLists update
* Added uv_safety script for teleop control of lights
* Cleanup
* uv_treatment simulation cleanup
  1. costmap - inflation radius reduced to 0.5
  2. topo_map for riseholme-uv_sim added
  3. scenario sim_riseholme-uv.sh updated with new TMAP and MAP values
  4. cleanup in uv_treatment.py
* cleanup and merging latest changes in master to uv related parameters
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* Merge branch 'master' of github.com:LCAS/RASberry into uv
* Merge pull request `#145 <https://github.com/LCAS/RASberry/issues/145>`_ from adambinch/master
  Merging this. costmap parameter modified and topo_map adjustment.
* Adding uv_rig on/off service calls
* Adjusted some of the costmap and local planner params (these were confirmed to work during a practical session with the robot at Riseholme.)
  Shifted end row nodes in topo map by 4cm in y direction. Adjusted position of node 10 in topo map for simulation.
* added new set of DWAPlannerRos parameters for uv treatment scenario
* Initial commit of rasberry_uv
* uv-sim
* RASberry setup to now uses carrot_planner by default
* Merge pull request `#143 <https://github.com/LCAS/RASberry/issues/143>`_ from gpdas/master
  Merging this. Tested. multiple odom frame publishers in simulation fixed
* setting enable_odom_tf to false in robot_bringup to avoid TF problems in simulation.
* Minor modification on SCENARIO riseholme-uv.sh
* Merge pull request `#142 <https://github.com/LCAS/RASberry/issues/142>`_ from gpdas/master
  Merging this. Topological map for uv-rig navigation in riseholme
* Topological map for uv-rig navigation in riseholme
* Merge pull request `#140 <https://github.com/LCAS/RASberry/issues/140>`_ from gpdas/master
  Merging this. Modifications to 007 config and sensor xacro
* Modifications to 007 config and sensor xacro
* Merge pull request `#137 <https://github.com/LCAS/RASberry/issues/137>`_ from adambinch/master
  Merging this. Config files for thorvald_006
* Config files for thorvald_006
* Merge pull request `#135 <https://github.com/LCAS/RASberry/issues/135>`_ from adambinch/master
  Merging this. New nodes added to topological map located at charging points in food…
* Merge branch 'master' into master
* Merge pull request `#134 <https://github.com/LCAS/RASberry/issues/134>`_ from Jailander/master
  Looks good. Merging this. Adding the possibility to use carrot planner for the uv scenario
* New nodes added to topological map located at charging points in food handling unit.
  Influence zone of node (waypoint) 68 has been decreased.
  Corresponding changes made to the topological map for the simulation.
  Precision for roll value of hokuyo placement in `/rasberry_bringup/urdf/sensors.xacro` has been increased.
  Templates for the thorvald_006 have been added. Note that these are just a copy of the ones for thorvald_007 at the moment.
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge pull request `#133 <https://github.com/LCAS/RASberry/issues/133>`_ from adambinch/master
  Merging this. Adjustment of influence zone of the node outside FHU for smoother turning - both riseholme and riseholme_sim tmaps.
* Adding the possibility to use carrot planner for the uv scenario
* Adjustment to one of the nodes in the topological map. Updated the rviz config so that the planner paths are the correct.
* Merge pull request `#132 <https://github.com/LCAS/RASberry/issues/132>`_ from adambinch/master
  Merging this. Hokuyo orientation modified. Map, no go map and topological map added for the new simulation environment.
* Map, no go map and topological map added for the new simulation environment.
  The plant models have been removed (for now) from this world to speed the sim up.
  I will look at ways of increasing the sim's real-time factor that will allow me to put them back in.
  `rasberry_bringup/launch/hokuyo.launch` and `rasberry_bringup/urdf/robot_007_sensors.xacro` have been updated
  due to the adjustment of the yaw of the hokuyo on the physical robot (it is now at zero degrees).
* Merge remote-tracking branch 'upstream/master'
* Tmule and Launch files for people perception (`#129 <https://github.com/LCAS/RASberry/issues/129>`_)
* make persistent topic persistent in ~/.ros
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
* riseholme coordinator is now LCASABU02 PC
* changes allow the sim to be run using the tmule set up
* removed outdated files
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
* no sleeps necessary with new tmule
* added scenario and more flexible tmule script
* more univeraal launch files
* added riseholme maps and uk robot 007 config files
* Merge pull request `#120 <https://github.com/LCAS/RASberry/issues/120>`_ from gpdas/master
  Merging this. Post Noway workshop demo Cleanup
* Fix exec_deped tag in rasberry_bringup/pacakge.xml
* CMakeLists.txt and package.xml updated with dependencies and install targets
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge branch 'master' into master
* Merge pull request `#119 <https://github.com/LCAS/RASberry/issues/119>`_ from tuandle/master
  Selectively choose config file for localization
* Merge branch 'master' of github.com:LCAS/RASberry
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
* selectively choose config file for localization base on usage of imu
* Merge pull request `#115 <https://github.com/LCAS/RASberry/issues/115>`_ from tuandle/master
  WIP properly fuse IMU measurements (including orientation, gyro and acceleration) for localization
* Merge branch 'master' into master
* Merge remote-tracking branch 'upstream/master'
* Merge pull request `#117 <https://github.com/LCAS/RASberry/issues/117>`_ from gpdas/master
  merging this. tmule config and coordination launch file cleanup
* properly fuse imu measurements
* rosbridge_ip and _port (of the coordination server) are configured as environment variables.
* Some cleanup after Norway Workshop demo
  1. rasberry-robot.yaml (tmule config) updated with mongodb_store launch
  2. rasberry-server.yaml (tmule config) updated with mongodb folder name. hyphen in the earlier foldername was causing some errors during mongodb_store.launch with an empty db.
  3. move_base_simple/goal is not exposed at the coordination server side
* Merge pull request `#2 <https://github.com/LCAS/RASberry/issues/2>`_ from LCAS/master
  Sync latest version of demo
* Merge pull request `#113 <https://github.com/LCAS/RASberry/issues/113>`_ from gpdas/master
  merging this. delay to other roslaunches in robot bringup config
* delay added to other roslaunches after roscore in rasberry-robot.yaml
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge branch 'master' of https://github.com/LCAS/RASberry
* removed duplicate map server
* wip to get robot up and running (`#112 <https://github.com/LCAS/RASberry/issues/112>`_)
  * tmux shell script updates
  * IMU EKF params correction. teleoperation enabled with robot launch
* Merge branch 'master' into master
* IMU EKF params correction. teleoperation enabled with robot launch
* tmux shell script updates
* Merge branch 'master' of https://github.com/LCAS/RASberry
* wip
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge pull request `#111 <https://github.com/LCAS/RASberry/issues/111>`_ from gpdas/master
  merging this. tweaks in the tmule configs
* 2d map server moved to robot tmule config, removed it from websocket_topological config
  removed server_websocket\_* (deprecated)
* Merge branch 'master' into master
* mpa must run locally
* minor
* wip
* wip
* first tmule for the server
* robot no
* New Config file and Velodyne Frame in xacro (`#109 <https://github.com/LCAS/RASberry/issues/109>`_)
* robot no
* typo
* fixed robot name
* tmule
* Merge pull request `#1 <https://github.com/LCAS/RASberry/issues/1>`_ from LCAS/master
  sync with LCAS
* initial version working on the robot
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
* Merge pull request `#104 <https://github.com/LCAS/RASberry/issues/104>`_ from gpdas/norway_rob_cfg
  Merging this. Mostly updatign Norway robot model dimensions and move_base parameters.
* XORG Display default changed to 0
* Norway robot configuration changes
  1. Robot dimensions adjusted -> rasberry_bringup/config/norway_robot.yaml
  2. sensor frames corrected -> rasberry_bringup/urdf/norway_robot_sensors.xacro
  3. robot_bringup.launch is added to bring the robot up (physical or simulated)
  4. rasberry_bringup/scripts/start_sim.sh modified with latest launch files
  5. move_base parameters adjusted to the ones used in the robot, including a new global_obstacle_layer in which laser scan is not used for marking/clearing costmap.
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#96 <https://github.com/LCAS/RASberry/issues/96>`_ from YiannisMenex/master
  Merging this. New Thorvald Model config with correct dimensions for Norway robot
* New Thorvald Model config
  Committing only the model file, as requested by @vigneshrajap.
  This was the fix @larsgrim provided, regarding the real robot losing orientation when perfmorming sharp turns.
* Multi thorvald simulations (`#85 <https://github.com/LCAS/RASberry/issues/85>`_)
  * Multi thorvald simulations
  Lauch files for multiple thorvalds added. The launch files launch two robots in their own namespaces. move_base works for both robots. This needs [Thorvald repo commit f73668c](https://github.com/LCAS/Thorvald/commit/f73668c280685e989d29a996693662058d16eec6) to work!
  1. thorvalds are named as `thorvald_001` and `thorvald_002`.
  2. Only move_base with teb local planner is tested.
  3. Similar to earlier simulations instructions, run start_sim.sh to start tmux session for this.
  4. map_server is moved out from move-base-teb launcher to an independent launch file
  * Fixed an issue with robot_pose_publisher not publishing
  1. robot_pose_publisher is launched from move_base launch files now, earlier it was in amcl.launch
  2. frames are properly set for robot_pose_publisher to publish robot_pose topics correctly
  * XORG DISPLAY is set to 0 now
  It was set to 1 earlier for my laptop.
  * Revert "Fixed an issue with robot_pose_publisher not publishing"
  This reverts commit e9ecad2c7a0ef35b1131958bb95f74b8910a78e7.
  * XORG DISPLAY is set to 0 now
  *  Fixed an issue with robot_pose_publisher not publishing
  1. robot_pose_publisher is launched from move_base launch files now, earlier it was in amcl.launch
  2. frames are properly set for robot_pose_publisher to publish robot_pose topics correctly
  * Multi-robot simulation setting initial pose in amcl
* Multi-robot simulation setting initial pose in amcl
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
* XORG DISPLAY is set to 0 now
* nw
* new_work
* XORG DISPLAY is set to 0 now
  It was set to 1 earlier for my laptop.
* Multi thorvald simulations
  Lauch files for multiple thorvalds added. The launch files launch two robots in their own namespaces. move_base works for both robots. This needs [Thorvald repo commit f73668c](https://github.com/LCAS/Thorvald/commit/f73668c280685e989d29a996693662058d16eec6) to work!
  1. thorvalds are named as `thorvald_001` and `thorvald_002`.
  2. Only move_base with teb local planner is tested.
  3. Similar to earlier simulations instructions, run start_sim.sh to start tmux session for this.
  4. map_server is moved out from move-base-teb launcher to an independent launch file
* First commit for topoNav testcases
* Merge pull request `#83 <https://github.com/LCAS/RASberry/issues/83>`_ from adambinch/master
  Corrected an error where actor laser data was failing to make it into…
* Corrected an error where actor laser data was failing to make it into the local cost map (actors now have dynamic obstacle avoidance).
  Actors no longer use amcl to localize - they now use their ground truth locations.
  To Do: actors avoid obstacles most of the time but tuning of the teb local planner needed. README.md needs updating.
* navigation launch
* Merge pull request `#74 <https://github.com/LCAS/RASberry/issues/74>`_ from adambinch/master
  Implemented move_base nav for multiple actors
* w.i.p.
* The package rasberry_actors has been subsumed into rasberry_gazebo.
  Move base has been implemented for multiple actors using teb local planner.
  A node has been created for publishing sequences of nav goals - can be used for actors or robots (see rasberry_gazebo/scripts/move_base_seq.py).
  A launch file for use with the physical robot (no sim elements) only has been made. See rasberry_gazebo/launch/thorvald_realtime.launch.
  To fix: actors only avoiding obstacles in the no go map. Probably laser data not making it into local costmap.
* correct maintainer
* Merge pull request `#70 <https://github.com/LCAS/RASberry/issues/70>`_ from LCAS/tmule
  first tmule script
* first tmule script
* Changes for testing (not to be merged)
* Contributors: Adam Binch, Alexander Gabriel, Gautham P Das, Jaime Pulido Fentanes, Johnmenex, LCASABU02, Marc Hanheide, ThomasDegallaix, Tuan Le, Vignesh, Yiannis Menexes, adambinch, gpdas, jailander, scosar, thorvald, thorvald007, tuandle

0.0.4 (2018-07-18)
------------------
* Added simulation with velodyne and kinect2, rviz config file to visualize them and changed actor to use gpu based simulated laser
* Changing topological map name
* adding simulation start-up script
* Contributors: Jaime Pulido Fentanes, mfernandezcarmona@lincoln.ac.uk

0.0.3 (2018-07-16)
------------------
* equal versions
* Can specify starting pose in rasberry_bringup/launch/rasberry_simulation.launch
* should work now ...
* Seperated the gazebo world with polytunnels and actor spawning into seperate packages (rasberry_gazebo and actor_gazebo, respectively).
  There is also a rasberry_bringup package that launches everything together
* Contributors: Marc Hanheide, adambinch

* equal versions
* Can specify starting pose in rasberry_bringup/launch/rasberry_simulation.launch
* should work now ...
* Seperated the gazebo world with polytunnels and actor spawning into seperate packages (rasberry_gazebo and actor_gazebo, respectively).
  There is also a rasberry_bringup package that launches everything together
* Contributors: Marc Hanheide, adambinch

0.0.2 (2018-05-21)
------------------

0.0.1 (2018-03-05)
------------------
