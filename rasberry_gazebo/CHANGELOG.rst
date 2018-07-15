^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
