^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_des
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2018-11-02)
------------------
* Merge pull request `#154 <https://github.com/LCAS/RASberry/issues/154>`_ from gpdas/master
  Merging this. Task cancellation & other updates in rasberry_coordination
* Picker can cancel task from CAR interface - rasberry_coordination
  1. CancelTask.srv from rasberry_coordination is removed. Instead strands_executive_msgs/CancelTask is used
  2. coordinator.py: Tasks can be cancelled from CAR interface
  - if the task is being processed, it will be cancelled and the robot will be sent to base
  - if the task is not yet taken for processing, it will be ignored when taken out of the queue
  - collect_tray failure case has been added to done_cb in coordination
  3. picker_state_monitor.py
  - task_time added to get the latest task of pickers
  - user_id replaced with picker_id to be consistent and to avoid mistakes, as both variable may have some value.
  - task_robot for the task will be None if the task had a robot assigned, but is cancelled
  4. robot.py
  - send the robot to base if current_goal is cancelled
* nw
* new_work
* First commit for topoNav testcases
* w.i.p.
* Contributors: Gautham P Das, ThomasDegallaix, adambinch, gpdas

0.0.4 (2018-07-18)
------------------
* Minor correction in polytunnel_des_config.yaml
* Adding white noise to introduce variations in picking and transportation rates
  Gaussian white noise is added to Picker's picking rate and transportation rate, and Robot's transportation rate. The noise is assumed to have zero mean and 2% of picking (or transportation) rate.
* Contributors: gpdas

0.0.3 (2018-07-16)
------------------

0.0.2 (2018-05-21)
------------------
* Merge pull request `#50 <https://github.com/LCAS/RASberry/issues/50>`_ from gpdas/master (Secondary head lane & Config file format changes)
  Tested OK.
  No major change in any agent behaviours.
* README updates
* Config file modifications
  Recent commits increased the number of config parameters and the parameters are no longer loaded to param server. The config files are modified to make it easier to work with these changes, by modifying parameters which needed another func parameter for populating to a larger list, are now dicts with func and value fields. config_utils.py is modified to address this.
  README updated
* README updated
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
* Polytunnel support
  Tested OK.
  Polytunnel support with some additional parameters:
  Some configuration parameters are added to decide how they should be treated.
  * Values for individual entities (rows, pickers, robots),
  * Copy of values to all entities,
  * Copy of values for rows in a polytunnel,
  * Values to be generated from a Gaussian distribution with given mean and std.
  No variation in the picking or transportation process. Main difference is in the map generation.
* Different polytunnels can have different number of rows
  Polytunnels can have different number of rows now
  - polytunnels_des_config.yaml is modified to run a scenario with this
  - Some more func parameters added
  - Other example configuration files are modified to reflect the changes in config parameters
  Simple Python Exceptions are raised now.
* Added polytunnel support in config file
  Parameters in config file are modified
  - some additional parameters (mainly func to replicate values to all rows / polytunnels / pickers)
  - picking_rate modified as picker_picking_rate
  - existing config files are updated with these changes
  - functions in config_utils.py are modified to address these
  - added a polytunnel_des_config.yaml (with corresponding launch and map files) as an example for polytunnel config
  DES changes corresponding to polytunnels
  - rows at the edges of each polytunnel are in half_rows
  - farm_rows modified in visualise.py (no farm row at intersection of polytunnels)
* In the config files, the parameter half_rows is removed.
  In topological graph, a new attribute half_rows added. This is the list of rows requiring picking in one direction.
  New maps are generated with these config files and are in maps.
  All scripts are modified to address this parameter changes.
* Merge pull request `#46 <https://github.com/LCAS/RASberry/issues/46>`_ from gpdas/master
  Tested and working fine.
* code cleanup
* Configuration changes:
  - added an optional config parameter "dist_to_cold_storage" to indicate distance from hn-00 to cold_storage
  - all farm and navigational rows are assumed to be aligned with x axis (was y axis earlier)
  - config_utils will read and make available dist-to_cold_storage in config_params
  visualise.py
  - alignment change in farm rows
* README.md
  - some corrections in the instructions
  des_config.yaml, open_field_des_config.yaml
  - config parameters values changed. robot speed reduced and tweaking some others
  des.py
  - code cleanup
  farm.py
  - scheduler now considers the new modes of picker and robot (when full trays need to be transported to cold storage, they may have to return to local storage)
  generate_map.py
  - add a cold_storage node at a given distance from hn-00 (head node of row_00)
  picker.py
  - now the crates may get unloaded at cold storage.
  - added an extra mode for the picker to return to local storage, when allocations are not complete, but current row is finished.
  - transport to cold storage is taken care using "use_local_storage" attribute from the topo_graph
  - code cleanup
  robot.py
  - now full trays may get unloaded at cold storage
  - added an extra mode for the robot to return to local storage for next assignment
  - transport to cold storage is taken care using "use_local_storage" attribute from the topo_graph
  - code cleanup
  topo.py
  - added a cold_storage_node and a method to set the resource
  visualise.py
  - class name changed
  - code cleanup
  - takes a few more arguments to save the figure canvas at some random points
* Merge branch 'master' of https://github.com/LCAS/RASberry
* Merge pull request `#43 <https://github.com/LCAS/RASberry/issues/43>`_ from gpdas/master
  Enable running DES in a loop
* Enable running the DES in a loop for different number of pickers and robots, different scheduling policies and for different number of trials
  config/des_config.yaml
  - Default des_env changed to simpy
  config/open_field_des_config.yaml
  - 29 farm rows with half rows (equivalent to 28 farm rows)
  - Node distance increased to 2.0 m to decrease number of events and thereby reducing sim time
  scripts/des.py
  - Added a variable for verbose passed to other agents. This now decides whether to print ros loginfos
  - Topological graph with the yields are created outside the loops once and used for all trials
  - Some config_params are re-read to get lists of size n_pickers and n_robots, which are updated in each loop
  - In a DES loop when there are no event scheduled next (env.peek() == inf), ros is not shutdown
  - Stats printing corrected
  src/rasberry_des/config_utils.py
  - get_des_config() now takes two more arguments, n_pickers and n_robots to override the parameters received from param server
  src/rasberry_des/farm.py
  - Takes another argument to control rospy loginfo
  - After loading trays on a robot, a picker to continue picking and the robot to go to local storage, confirmation from the scheduler is needed. This ensures there is at most one robot assigned to a picker at any time. After scheduler is informed that the trays are loaded, the picker is removed from waiting_for_robot_pickers. Once the picker resumed picking or changed to any other mode, the assigned_picker_robot[picker_id] is made None.
  - bugs fixed in shortest_distance and utilise_all scheduling policies for both row_allocations and robot_assignments. The idea is to update the idle_picker/idle_robots list after each assignment and sort them based on the criteria (shortest, less work) and always assign the one at the starting.
  - bugs fixed where the iterable used for looping was modified from within the loop resulting in unfavourable situations. The pickers/robots to be removed are noted down and are removed after completing the main loop with the iterable.
  src/rasberry_des/picker.py
  - Takes another argument to control rospy loginfo
  - Removed wait_out method to reduce number of events
  - After loading trays, the picker waits for continue_picking to be set by the scheduler (by calling proceed_with_picking)
  src/rasberry_des/robot.py
  - Takes another argument to control rospy loginfo
  - Removed wait_out method to reduce number of events
  - After trays are loaded, the robot waits for continue_transporting to be set by the scheduler (by calling proceed_with_transporting)
  src/rasberry_des/topo.py
  - Takes another argument to control rospy loginfo
  - local_storages are set by calling set_local_storages externally. This is to enable running the DES in a loop, where the default capacity of local storages are modified in each loop.
  src/rasberry_des/visualise.py
  - code cleanup
  bug fixing going on for robot assignment after the current robot is loaded
  looping in des.py
* launch file rename
* README
* Minor tweaks in save_stats in des.py
  time_spent\_* corrections in picker and robot
* Merge pull request `#42 <https://github.com/LCAS/RASberry/issues/42>`_ from gpdas/master
  DES with both pickers, robots and visualisation
* Merge branch 'robot_agents_no_ros'
* Changes in readme
* config and launch config launch files for open_field added
  code cleanup
* des.py - Code cleanup
  farm.py, picker.py, robot.py, visualise.py
  - simpy process exit conditions
  - rospy logging
* des.py
  - removed parameter des_running
  - Farm, Picker, Robot and TopologicalForkMap object argument changes
  - order of creation: TopologicalForkMap, Robot, Picker, and Farm
  - cleanup in picking information printing
  farm.py
  - takes Picker and Robot objects, instead of their ids
  - removed simpy events for row_completion
  - farm object is not available to pickers and robots
  - specific information passing from scheduler to pickers or robots is through special methods in those.
  - removed all ros services and action clients
  - scheduling loop recoded without ros services and simpy events
  picker.py
  - code cleanup
  robot.py
  - code cleanup
* Removed msg, srv and action - modified CMakeLists and package
  Picker and Robot classes are nearly 100% ready for des with no ros usage between agents
* Removing all ros related msgs srvs actions. State changes remain the same.
  Farm - 95% complete
  Picker - 80%
  Farm - 5%
* Stripping down most ros related calls - publishers and subscribers in this commit
  Visualise_Agents takes robots and pickers and gets their poses and statuses from the agent objects
* des.py
  - signals ros shutdown to close all ros related background threads (Program not exiting without this)
  - figure is closed from here
  - added a new rosparam 'des_running' to stop normal_operation of the robot agents. set from here
  farm.py
  - trays_unload -> trays_unloaded
  - tray_loaded -> trays_loaded
  - robot_info is modified to exit only after assigning a robot
  - des_running is reset from here after all rows are picked
  picker.py
  - trays_unload -> trays_unloaded
  - no. of trays are modified after unloading
  robot.py
  - tray_loaded -> trays_loaded
  - collectionGoal object is not needed as action server execute_cb is completed in a single method
  - des_running to stop normal_operation of robot
  - calls to other functions from the execute_cb are not working, and goal was set to aborted. so all functionality now in a single method. all yields required in simpy had to be removed.
  - feedback.storage_node -> feedback.local_storage_node
  visualise.py
  - method to close the figure
* Extrapolated pose readings are removed
* renames pickers_only .py and .launch files
* Robot_Collection.action modified
  des_config.yaml modified. Some params are configured to take a list of values, say of length n_pickers etc., take two values for gauss,or single value for copy
  config_utils.py modified to address changes in des_config.yaml
  CMakeLists.txt modified with Robot_Info.srv
  Robot_Status.msg Robot_Info.srv - field tweaks and fixes
  pickers_only.py, visualise.py, topo.py, generate_map.py - code cleanup
  Farm - robot assigned in the callback of Robot_Info, code cleanup for brevity and fixes
  Picker - code cleanup, missing function dist_to_robot added
  Robot - code cleanup
* Modifications in messages, services and actions
  Farm, Picker and Robot classes are modified for action feedbacks
* Add another simpy process in Picker to work with robot carriers
* topo_graph is an object outside the farm and is passed to all other relevant classes
  Robot agents not yet functional
* New action for collection and unload for the robots
  New service for pickers to report tray full and for unloading trays
  New config parameter robot_max_n_trays
  Dependencies in package.xml and CMakeLists.txt
  Minor modifications in Picker and Farm
  Robot is modified - not in a working state though
* Farm gets pose and status of all pickers and robots now
  Some configuration parameters are renamed / added
  Robot_Status.msg has additional field
  Code cleanup in Visualise_Agents, Picker, Robot and Farm
* Added mode to Picker_Status.msg
  Basic Robot agent in robot.py
  Missing dependencies added
* Minor tweak in the picker_status message
* Merge pull request `#39 <https://github.com/LCAS/RASberry/issues/39>`_ from gpdas/master
  Visualising pickers
* Merge branch 'visualise_pickers'
* Picker now publishes status of picking progress using Picker_Status.msg
  Removed start_sim config parameter
  Code cleanup in pickers.py for new topics and bug fix in case of full_rows
  Visualisation of pickers added
  Dependencies updated
* All calls to tmap_utils is done through wrapper methods in TopologicalForkGraph
  Added a new config param start_sim to control the simulation
  Picker now publishes pose more often - to enable better visualisation
  Instead of a single timeout between each nodes, small timeouts are performed now
* Default value of half_rows is changed and map files are updated.
  A bug in selecting x position of the nodes during fork_map generation is fixed
  A new class for visualising the dynamic objects in a matplotlib plot is in visualise.py
* Deleted some old classes.
* Merge pull request `#38 <https://github.com/LCAS/RASberry/issues/38>`_ from gpdas/master
  Basic support for topological_navigation in the DES with pickers alone
* Small changes in pickers_only.py to print the allocation and picking details after finishing all other things.
  rospy.loginfo is used instead of prints in most places
* minor change in README
* Now topological map stored in the mongodb, and the topics and services provided by topological navigation nodes are used for all route calculations in the simulation. Removed the Graph and Node classes defined earlier in topo.py.
  Added a new configuration parameter for n_local_storages
* topological navigation related launch files are removed from the package and the original launch files in the topological_navigation package are called.
  configuration parameters for the des are launched along with a map_server and static_transform_publisher nodes in rasberry_des_config.launch (these were part of the topological_navigation launch files in the package earlier)
  picker_id has "_" instead of "-" in picker_only.py
  README updated
* CMakeList is modified with the launch files
* removed tf2_broadcaster. static_transform_publisher is used.
* picker respect half or full rows now.
* namespace of config parameters is not sent as an argument anymore. assumes ns/rasberry_des_config/<param_name>
* deleted couple of backup files.
* Merge branch 'master' of https://github.com/LCAS/RASberry into topo_nav
* An initial step towards adding topological map layer for the navigation within the
  farm.
  All configurations are now moved into a yaml file, which is loaded along with some topological_navigation nodes.
  > des_env : simulation environment
  > n_farm_rows and n_topo_nav_rows are now different
  > half_rows or full_rows at either ends can now be configured
  > head node to first node distance in each row can be configured
  > head node y coordinate can be configured for each row
  > spacing between different rows can be configured
  > picking rate for each picker can be configured or an average can be given (a normally distributed value is set to each picker)
  > transportation rate for each picker can be configured or an average can be given (a normally distributed value is set to each picker)
  > loading time for each picker can be configured or an average can be given (a normally distributed value is set to each picker)
  > max_n_trays can be specified for each picker or for all pickers
  > yield per row node can be configured for each row or for all rows. A logistic distribution is assumed for the yield.
  Some topological_navigation nodes are also initialised and fork_map is loaded to the mongodb.
  How to run: check README.md
* Contributors: Gautham P Das, adambinch, gpdas

0.0.1 (2018-03-05)
------------------
* Modified the Picker class to publish /<picker_name>/pose (geometry_msgs.msg.Pose) topic when it reaches a node.
  Ros topics were not published while runnning quick sim (simpy.Environment), probably too fast. This needs double checking.
* Changes:
  1. env.step() is called in a while loop checking rospy.is_shutdown(), rather than env.run().
  2. A bug in the Picker is fixed. The picker no longer re-pick the same row, after it is completed and scheduler_monitor process has not allocated a new row.
* Change(s):
  1. Modified into a rospackage with one node pickers_only.py
  2. Node initialisation is the only ros functionality at this stage.
  3. Farm and Picker classes defined in pickers_only.py are moved into individual files(farm.py and picker.py)
  Known Issue(s):
  1. SimPy processes are not interrupted by Ctrl+c killing the node.
* Initial commit of the discrete event simulation of a strawberry farm.
  This simulates only pickers and a farm allocation monitoring process.
* Contributors: gpdas
