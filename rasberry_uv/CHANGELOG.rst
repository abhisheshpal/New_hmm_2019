^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_uv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#149 <https://github.com/LCAS/RASberry/issues/149>`_ from gpdas/master
  Merging this. Package for autonomous UV treatment - initial commits
* Updating deps and install targets
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* fixing the launch file
* Added uv_safety script for teleop control of lights
* uv_treatment simulation cleanup
  1. costmap - inflation radius reduced to 0.5
  2. topo_map for riseholme-uv_sim added
  3. scenario sim_riseholme-uv.sh updated with new TMAP and MAP values
  4. cleanup in uv_treatment.py
* clean up and topo_map updates in uv_treatment
* uv_rig now returns a response
* cleanup and merging latest changes in master to uv related parameters
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* Adding uv_rig on/off service calls
* cleanup rasberry_uv/uv_treatment.py
* Initial commit of rasberry_uv
* Contributors: Gautham P Das, LCASABU02, Marc Hanheide, adambinch, gpdas, thorvald007

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
* Merge pull request `#149 <https://github.com/LCAS/RASberry/issues/149>`_ from gpdas/master
  Merging this. Package for autonomous UV treatment - initial commits
* Updating deps and install targets
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* fixing the launch file
* Added uv_safety script for teleop control of lights
* uv_treatment simulation cleanup
  1. costmap - inflation radius reduced to 0.5
  2. topo_map for riseholme-uv_sim added
  3. scenario sim_riseholme-uv.sh updated with new TMAP and MAP values
  4. cleanup in uv_treatment.py
* clean up and topo_map updates in uv_treatment
* uv_rig now returns a response
* cleanup and merging latest changes in master to uv related parameters
* Merge branch 'uv' of github.com:gpdas/RASberry into uv
* Adding uv_rig on/off service calls
* cleanup rasberry_uv/uv_treatment.py
* Initial commit of rasberry_uv
* Contributors: Gautham P Das, LCASABU02, Marc Hanheide, adambinch, gpdas, thorvald007

0.0.4 (2018-07-18)
------------------

0.0.3 (2018-07-16)
------------------

0.0.2 (2018-05-21)
------------------

0.0.1 (2018-03-05)
------------------
