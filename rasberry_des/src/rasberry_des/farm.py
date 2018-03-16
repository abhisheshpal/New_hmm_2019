#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import simpy
import topo


class Farm(object):
    """Farm class definition"""
    def __init__(self, name, env, n_farm_rows, half_rows, _yield_per_node, local_storages):
        """Create a Farm object

        Keyword arguments:
        name -- name of farm/poly-tunnel
        env -- simpy.Environment
        """
        self.name = name
        self.env = env

        # pickers should report for duty.
        # could be useful in future to take breaks or late arrivals
        self.pickers_reported = []

        self.n_farm_rows = n_farm_rows
        self.half_rows = half_rows
        self.n_topo_nav_rows = n_farm_rows - 1 if half_rows else n_farm_rows + 1
        self.row_ids = ["row-%02d" %(i) for i in range(self.n_topo_nav_rows)]              # can be a row name / id

        # related to picker finishing a row
        # finished_rows - A dict of simpy.Events. picker triggers
        self.finished_rows = {row_id:simpy.Event(self.env) for row_id in self.row_ids}
        self.n_finished_rows = 0
        self.row_finish_time = {row_id:None for row_id in self.row_ids} # {row_id: finish time}

        # related to allocation
        self.unallocated_rows = [] + self.row_ids                       # a list of unallocated rows
        self.allocations = {row_id:None for row_id in self.row_ids}     # {row_ids: picker_id}
        self.allocation_time = {row_id:None for row_id in self.row_ids} # {row_id: allocation time}

        self.picker_allocations = {}        # {picker_id:[row_ids]}
        self.curr_picker_allocations = {}   # {picker_id:row_id} row=None if free

        # topological map based graph ()
        self.graph = topo.TopologicalForkGraph(self.n_topo_nav_rows, self.row_ids, _yield_per_node, local_storages)

    def picker_report(self, picker_id):
        """Method a picker should call when he reports to work

        Keyword arguments:
        picker_id -- picker's unique id
        """
        if picker_id not in self.pickers_reported:
            self.picker_allocations[picker_id] = []
            self.curr_picker_allocations[picker_id] = None
            self.pickers_reported.append(picker_id)
            print("%s reporting at %0.3f" %(picker_id, self.env.now))

    def finished_picking(self, ):
        """Method to check whether all allocated rows are finished"""
        # method to check whether all rows are picked.
        # return True or False, based on picking is finished or not
        if self.n_finished_rows == self.n_topo_nav_rows:
            return True
        return False

    def finished_allocating(self, ):
        """Method to check whether all rows are allocated"""
        # method to check whether all rows are allocated.
        if self.unallocated_rows:
            return False
        return True

    def scheduler_monitor(self, ):
        """A process to allocate rows to the pickers.
        the picker should request for a row or
        when a picker becomes free, it should be allocated automatically.

        A simple implementation:
            external: picker triggers the finishedRows[row_id] event,
                        when the allocated row is completed
            1. do a periodic checking of completion status of rows
            2. allocate free pickers to one of the unallocated rows
        """
        while True:
            # check for any completion update of any rows
            for i in range(len(self.pickers_reported)):
                picker_id = self.pickers_reported[i]
                if self.curr_picker_allocations[picker_id] is not None:
                    # check if there is a row allocated to the picker
                    row_id = self.curr_picker_allocations[picker_id]
                    if self.finished_rows[row_id].triggered:
                        # this row is finished
                        self.n_finished_rows += 1
                        # TODO: there could be slight delay in this time
                        print ("%s reported finish-row at %0.3f and now time is %0.3f" %(picker_id,
                                                                                           self.finished_rows[row_id].value,
                                                                                           self.env.now))
                        self.row_finish_time[row_id] = self.finished_rows[row_id].value
                        # relieve the picker
                        self.curr_picker_allocations[picker_id] = None
                        print("%s reported completion of row %s at %0.3f" %(picker_id,
                                                                             row_id,
                                                                             self.row_finish_time[row_id]))
            if self.finished_picking():
                # once all rows are picked finish the process
                break

            # allocate, if there are rows yet to be allocated
            if not self.finished_allocating():
                for picker_id in self.pickers_reported:
                    if not self.curr_picker_allocations[picker_id]:
                        # allocate if picker is free
                        # get the first free row
                        row_id = self.unallocated_rows.pop(0)
                        # allocate row_id to the picker
                        self.allocations[row_id] = picker_id
                        self.picker_allocations[picker_id].append(row_id)
                        self.allocation_time[row_id] = self.env.now
                        # the value picker checks is updated last
                        self.curr_picker_allocations[picker_id] = row_id
                        print("%s is allocated to %s at %0.3f" %(picker_id,
                                                                  row_id,
                                                                  self.allocation_time[row_id]))

                    # check if all rows are allocated after each allocation
                    if self.finished_allocating():
                        break

            # take a short break after each allocation loop
            yield self.env.timeout(0.1)

