^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_people_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge remote-tracking branch 'upstream/master'
* Tmule and Launch files for people perception (`#129 <https://github.com/LCAS/RASberry/issues/129>`_)
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
* Merge pull request `#120 <https://github.com/LCAS/RASberry/issues/120>`_ from gpdas/master
  Merging this. Post Noway workshop demo Cleanup
* CMakeLists.txt and package.xml updated with dependencies and install targets
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
* Merge pull request `#2 <https://github.com/LCAS/RASberry/issues/2>`_ from LCAS/master
  Sync latest version of demo
* Merge pull request `#114 <https://github.com/LCAS/RASberry/issues/114>`_ from gpdas/master
  merging this. fixes to imu and marvel to topo localisation
* missing closest_node_msgs created in rasberry_people_perception/src/rasberry_people_perception/picker_marvel_localiser.py
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
* had to move package
* submodule marvelmind
* Merge branch 'master' into norway_rob_cfg
* Merge pull request `#100 <https://github.com/LCAS/RASberry/issues/100>`_ from gpdas/people_perception
  WIP: People perception
* fixes for build fail
  renamed scripts to script
  minor tweaks
* Merge pull request `#1 <https://github.com/LCAS/RASberry/issues/1>`_ from Khan1988/gpd_pp
  Transformation btw map and marvel mind is disable now
* Transformation btw map and marvel mind is disable now
  publishes posestamped and closest and current nodes of each picker
* Initial implementation of picker position publishers from ultrasonic targets
  A localiser node that
  - listens to hedge_pos_a topics from marvelmind_nav/hedge_rcv_bin node
  - transforms to a global frame and publishes posestamped
  - finds current & closest node in a topological map and publishes them
* Merge branch 'master' into master
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#93 <https://github.com/LCAS/RASberry/issues/93>`_  Initial commit of people_perception
  Merging this. needed package with a lunch file for ulstrsound localisation
* Initial commit of people_perception
  Added a launch file to launch marvelmind_nav nodes to publish ultrasonic localisation
* Contributors: Gautham P Das, Marc Hanheide, Tuan Le, adambinch, gpdas, khan1988, scosar, thorvald

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
* Merge remote-tracking branch 'upstream/master'
* Tmule and Launch files for people perception (`#129 <https://github.com/LCAS/RASberry/issues/129>`_)
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
* Merge pull request `#120 <https://github.com/LCAS/RASberry/issues/120>`_ from gpdas/master
  Merging this. Post Noway workshop demo Cleanup
* CMakeLists.txt and package.xml updated with dependencies and install targets
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
* Merge pull request `#2 <https://github.com/LCAS/RASberry/issues/2>`_ from LCAS/master
  Sync latest version of demo
* Merge pull request `#114 <https://github.com/LCAS/RASberry/issues/114>`_ from gpdas/master
  merging this. fixes to imu and marvel to topo localisation
* missing closest_node_msgs created in rasberry_people_perception/src/rasberry_people_perception/picker_marvel_localiser.py
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
* had to move package
* submodule marvelmind
* Merge branch 'master' into norway_rob_cfg
* Merge pull request `#100 <https://github.com/LCAS/RASberry/issues/100>`_ from gpdas/people_perception
  WIP: People perception
* fixes for build fail
  renamed scripts to script
  minor tweaks
* Merge pull request `#1 <https://github.com/LCAS/RASberry/issues/1>`_ from Khan1988/gpd_pp
  Transformation btw map and marvel mind is disable now
* Transformation btw map and marvel mind is disable now
  publishes posestamped and closest and current nodes of each picker
* Initial implementation of picker position publishers from ultrasonic targets
  A localiser node that
  - listens to hedge_pos_a topics from marvelmind_nav/hedge_rcv_bin node
  - transforms to a global frame and publishes posestamped
  - finds current & closest node in a topological map and publishes them
* Merge branch 'master' into master
* Merge branch 'master' of github.com:LCAS/RASberry
* Merge pull request `#93 <https://github.com/LCAS/RASberry/issues/93>`_  Initial commit of people_perception
  Merging this. needed package with a lunch file for ulstrsound localisation
* Initial commit of people_perception
  Added a launch file to launch marvelmind_nav nodes to publish ultrasonic localisation
* Contributors: Gautham P Das, Marc Hanheide, Tuan Le, adambinch, gpdas, khan1988, scosar, thorvald

0.0.4 (2018-07-18)
------------------

0.0.3 (2018-07-16)
------------------

0.0.2 (2018-05-21)
------------------

0.0.1 (2018-03-05)
------------------
