**rasberry_des**
------------
A rospackage for running discrete event simulation of a strawberry farm, pickers and a basic job allocation processesusing SimPy. It does not rely on any ros related functionalities yet, other than initialising the node.

# Nodes:
  1. pickers_only.py

     - `simpy.Environment` is used in `pickers_only.py` to run a quick simulation. `simpy.RealtimeEnvironment` can also be used to simulate in real-time (that line is commented in `pickers_only.py`).
     - `env.step` is used for getting a control over the simulation.


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

# Known Issues:
  1. `env.step` is used now rather than the `env.run` to kill the simulation with `Ctrl+c`. Although it gives better control over the simulation, 
     - some delay between the two clocks at the start. SimPy clock does not progress until the first `env.step`, but the all required process should be initialised before that.
     - Some delay (1-2 ms in ros/real-time clock) is observed between multiple events scheduled at the same simulation time.
     - However, chances of having multiple events scheduled at the same instance will be small except at the start.
     - This delay does not seem to build up over time.

# TODO:
  1. The topological map should be replaced with the `topological_navigation` from `strands_project/strands_navigation` to simulate robot processes. More complex map may be defined in future using this. For example, long rows with two head lanes at both ends. A row may be assigned to two pickers who will start from either end, with associated local storage on the head lane on that side.
  2. The fork-map generation should be done outside the `Farm` class and should only be read from a yaml file.
  3. Simulating robot process and integrate with the robots in gazebo
  4. Simulating ros-compatible picker processes and integrate with people movements in gazebo

