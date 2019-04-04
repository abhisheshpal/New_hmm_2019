#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_des.farm


class FarmMimic(rasberry_des.farm.Farm):
    """Farm class definition"""
    def __init__(self, name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, with_robots, n_iters=0, verbose=False):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        n_topo_nav_rows -- navigational rows in topological map
        topo_map -- TopologicalForkMap object
        robots - robot agent objects
        pickers -- picker agent objects
        policy -- "lexicographical", "shortest_distance", "uniform_utilisation"
        """
        super(FarmMimic, self).__init__(name, env, n_topo_nav_rows, topo_graph, robots, pickers, policy, verbose)

        self.with_robots = with_robots

        self.n_iters = n_iters
        self.curr_iter = 0
        self.curr_picker_iter = {picker_id:self.curr_iter for picker_id in self.picker_ids}
        self.curr_row_iter = {row_id:self.curr_iter for row_id in self.graph.row_ids}


    def check_iter_increment(self, ):
        """checks whether incrementing the iteration number for all rows is possible
        """
        # increment all idle picker's iter by one
        for picker_id in self.picker_ids:
            if self.pickers[picker_id].curr_mode == 0 and self.curr_picker_iter[picker_id] < self.curr_iter:
                self.curr_picker_iter += 1

        # This is possible when all
        pass

    def scheduler_monitor(self, ):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        inform_allocation_finished = False
        inform_picking_finished = False
        while True:
            if rospy.is_shutdown():
                break

            if self.finished_picking() and not inform_picking_finished:
                inform_picking_finished = True
                self.loginfo("all rows are picked")
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_picking_finished()
                self.loginfo("all rows picked. scheduler exiting")
                self.env.exit("all rows are picked")
                break

            if self.finished_allocating() and not inform_allocation_finished:
                self.loginfo("all rows are allocated")
                inform_allocation_finished = True # do it only once
                for picker_id in self.picker_ids:
                    self.pickers[picker_id].inform_allocation_finished()

            to_remove_pickers = []
            # update modes of pickers already assigned to a row
            for picker_id in self.allocated_pickers:
                picker = self.pickers[picker_id]
                if picker.mode == 0:
                    # finished the assigned row and are idle now
                    # if previously assigned any row, update its status
                    row_id = self.curr_picker_allocations[picker_id]
                    self.finished_rows.append(row_id)
                    self.n_finished_rows += 1
                    self.row_finish_time[row_id] = self.pickers[picker_id].row_finish_time
                    to_remove_pickers.append(picker_id)

                elif picker.mode == 1:
                    # moving to a row_node possibly from the previous node
                    # this can heppen either after a trip to a storage or after a new row allocation
                    # picker will be in picking mode (2) soon
                    pass

                elif picker.mode == 2:
                    # picking now
                    pass

                elif picker.mode == 3 or picker.mode == 4 or picker.mode == 6:
                    # picker transporting to storage or unloading at storage
                    # or transporting to local storage from cold storage
                    # if the current row is finished, the picker's mode will be changed
                    # to idle (0) soon, which will be taken care of in next loop
                    pass

                elif picker.mode == 5:
                    # waiting for a robot to arrive
                    # if a robot is not assigned, coordinator will assign one
                    pass

            for picker_id in to_remove_pickers:
                self.allocated_pickers.remove(picker_id)
                if self.pickers[picker_id].mode == 0:
                    self.idle_pickers.append(picker_id)

            # row allocation to idle_pickers
            self.allocate_rows_to_pickers()

            yield self.env.timeout(self.loop_timeout)

        yield self.env.timeout(self.process_timeout)