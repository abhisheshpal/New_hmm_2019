^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rasberry_des
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
