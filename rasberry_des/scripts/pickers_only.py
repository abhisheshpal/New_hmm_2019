#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 05-Feb-2018
# @info: Farm - a simple strawberry farm class
#        Picker - a simple picker class
#        Uses simpy to simulate three processes
#           1. Farm.scheduler_monitor()
#           2. Picker.picking_process()
#           3. Picker.transport_process()
#        Local storage is a simpy.Resource, now with the capacity same as N_PICKERS
#        Uses the simple topological graph representation in topo.py
# ----------------------------------

import random
import simpy
import numpy
import topo


RANDOM_SEED = 1234

N_ROWS = 3                  # number of rows
ROW_NODE_DIST = 2           # m, distance between two ndoes in a row
ROW_LENGTH = 4              # m, length of a row
ROW_SPACING = 2             # m, spacing between two rows

N_PICKERS = 3               # total number of pickers
PICKING_RATE = 0.2          # m/s, speed of the picker while picking
TRANSPORT_RATE = 0.8        # m/s, speed of the picker while transporting
MAX_N_TRAYS = 1             # maximum number of trays that can be carried by the
                            # picker
LOADING_TIME = 80.0         # s, time to be spent at the localStorage
TRAY_CAPACITY = 12 * 250.0  # g, tray is assumed to take 12 small trays, each 250 g

YIELD_PER_NODE = 200.0      # g/m, yield per node distance


class Farm(object):
    """Farm class definition"""
    def __init__(self, name, env):
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

        self.rows = []              # can be a row name / id
        self.n_rows = 0

        self.head_node_names = []   # [head_node_names]
        self.row_node_names = {}    # {row_id:[row_node_names]}

        # {"hn":row_spacing, row_id:[head_node, start_node, end_node,
        # row_node_dist, last_node_dist, local_storage]}
        self.row_info = {}

        # related to picker finishing a row
        self.finished_rows = {}     # a dict of simpy.Events. picker triggers
        self.n_finished_rows = 0
        self.row_finish_time = {}   # {row_id: finish time}

        # related to allocation
        self.unallocated_rows = []          # a list of unallocated rows
        self.allocations = {}               # {row_ids: picker_id}
        self.allocation_time = {}           # {row_id: allocation time}
        self.picker_allocations = {}        # {picker_id:[row_ids]}
        self.curr_picker_allocations = {}   # {picker_id:row_id} row=None if free

        self.graph = None

        self.local_storages = {}  # simpy.Resource objects
        self.local_storage_nodes = []     # local storage node

        print("""Initialise the farm-path graph using
                "init_graph" or "init_graph_fork"
        """)

    def picker_report(self, picker_id):
        """Method a picker should call when he reports to work

        Keyword arguments:
        picker_id -- picker's unique id
        """
        if picker_id not in self.pickers_reported:
            self.picker_allocations[picker_id] = []
            self.curr_picker_allocations[picker_id] = None
            self.pickers_reported.append(picker_id)
            print("%s reporting at %04.3f" %(picker_id, self.env.now))

    def finished_picking(self, ):
        """Method to check whether all allocated rows are finished"""
        # method to check whether all rows are picked.
        # return True or False, based on picking is finished or not
        if self.n_finished_rows == self.n_rows:
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
                        print ("%s reported finish-row at %04.3f and now time is %04.3f" %(picker_id,
                                                                                           self.finished_rows[row_id].value,
                                                                                           self.env.now))
                        self.row_finish_time[row_id] = self.finished_rows[row_id].value
                        # relieve the picker
                        self.curr_picker_allocations[picker_id] = None
                        print("%s reported completion of row %s at %04.3f" %(picker_id,
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
                        print("%s is allocated to %s at %04.3f" %(picker_id,
                                                                  row_id,
                                                                  self.allocation_time[row_id]))

                    # check if all rows are allocated after each allocation
                    if self.finished_allocating():
                        break

            # take a short break after each allocation loop
            yield self.env.timeout(0.1)

    def init_graph(self, f_name=None):
        """Generate a graph from the given file"""
        # TODO: fill here with code to read the file and generate the graph
        pass

    def init_graph_fork(self, n_rows, row_node_dist, row_length,
                        row_spacing, yield_per_node, local_storages):
        """Initialise a fork shaped graph

        Keyword arguments:
        n_rows -- number of rows
        row_node_dist -- distance between two row_nodes in each row.
                        can be a list in the order of row_id, a float or an int
        row_length -- length of each row.
                        can be list in the order of row_id, a float or an int
        row_spacing -- spacing between two rows
        yield_per_node -- float in g/m
        """
        self.n_rows = n_rows
        self.row_info["hn"] = row_spacing

        self.rows = ["row-%02d" %(i) for i in range(self.n_rows)]   # can be a row name / id
        self.unallocated_rows = [] + self.rows

        self.finished_rows = {row_id:simpy.Event(self.env) for row_id in self.rows}
        self.row_finish_time = {row_id:None for row_id in self.rows}

        self.allocations = {row_id:None for row_id in self.rows}
        self.allocation_time = {row_id:None for row_id in self.rows}

        if row_node_dist.__class__ == list:
            row_node_dist = {self.rows[i]:row_node_dist[i] for i in range(self.n_rows)}
        elif (row_node_dist.__class__ == float) or (row_node_dist.__class__ == int):
            row_node_dist = {row_id:float(row_node_dist) for row_id in self.rows}
        else:
            raise TypeError("row_node_dist must be list, float or int")

        if row_length.__class__ == list:
            pass
        elif (row_length.__class__ == float) or (row_length.__class__ == int):
            row_length = {row_id:float(row_length) for row_id in self.rows}
        else:
            raise TypeError("row_length must be list, float or int")

        last_node_dist = {row_id:0. for row_id in self.rows}

        self.graph = topo.BiGraph()

        row_nodes = {}
        head_nodes = {}

        # TODO: There can be more than one local storage nodes
        # As there are no guidelines about this at this stage, only one storage node is added
        # It is assumed that there is only one simpy.Resource object in local_storages
        # Place it at int(n_rows/2)
        self.local_storage_nodes.append("hn-%02d" %(int(round(self.n_rows / 2))))
        self.local_storages[self.local_storage_nodes[0]] = local_storages[0]

        # 1. create the nodes - head_nodes and row_nodes
        i = 0   # head node counter: one head node per row
        for row_id in self.rows:
            # All head nodes are Node objects with zero yield
            # All row nodes are Node objects with a yield_at_node
            x = i * row_spacing
            y = -5
            head_node_name = "hn-%02d" %(i)
            head_nodes[head_node_name] = topo.Node(head_node_name, x, y)

            self.head_node_names.append(head_node_name)
            self.row_node_names[row_id] = []

            # row length can be different for different rows
            # 1 is for the end node, which is not produced in numpy.arange
            n_row_nodes = len(numpy.arange(0, row_length[row_id], row_node_dist[row_id])) + 1
            for j in range(n_row_nodes):
                # row_length may not be an exact multiple of row_node_dist
                y = j * row_node_dist[row_id] if (j != n_row_nodes - 1) else row_length[row_id]
                # yield from each row node to next row node
                if j != n_row_nodes - 1:
                    yield_at_node = numpy.random.logistic(yield_per_node)
                else:
                    # between the last two nodes, the distance could be smaller than node_dist
                    last_node_dist[row_id] = row_length[row_id] - (row_node_dist[row_id] * (j-1))
                    yield_at_node = numpy.random.logistic((yield_per_node *
                                                           last_node_dist[row_id]) / row_node_dist[row_id])
                row_node_name = "rn-%02d-%02d" %(i, j)
                row_nodes[row_node_name] = topo.Node(row_node_name, x, y, yield_at_node)
                self.row_node_names[row_id].append(row_node_name)

            # info about the row - for assigned picker
            self.row_info[row_id] = [head_node_name,
                                     self.row_node_names[row_id][0],
                                     self.row_node_names[row_id][-1],
                                     row_node_dist[row_id],
                                     last_node_dist[row_id],
                                     self.local_storage_nodes[-1]]
            # increment head node counter
            i += 1

        # 2. find the neighbours of each node
        # 3. add nodes to the graph with corresponding edges

        # for each row, head node first and then the row nodes in that row
        i = 0   # head node counter
        for row_id in self.rows:
            # neighbours of head nodes
            head_node_neighbours = []
            row_node_neighbour = []

            head_node_name = "hn-%02d" %(i)
            # neighbouring head nodes
            head_node_neighbours = []
            if (i == 0) and (self.n_rows == 1):
                # if the only head node, there are no head node neighbours
                pass
            elif (i == 0) and (self.n_rows != 1):
                # more than one head node, add the next head node
                head_node_neighbours.append(head_nodes["hn-%02d" %(i + 1)])
            elif i == self.n_rows - 1:
                # for the last node (there would be at least 2 head nodes, bcz of conditions above)
                head_node_neighbours.append(head_nodes["hn-%02d" %(i - 1)])
            else:
                # for any other head node, add the previous and next head nodes as neighbours
                head_node_neighbours.append(head_nodes["hn-%02d" %(i - 1)])
                head_node_neighbours.append(head_nodes["hn-%02d" %(i + 1)])
            # row node neighbours
            # there would be only one row node as neighbour to the head node of the row
            row_node_neighbour = [row_nodes["rn-%02d-%02d" %(i, 0)]]
            # add i-th head nodes
            self.graph.add_node(head_nodes[head_node_name],
                                head_node_neighbours + row_node_neighbour)

            # neighbours of row nodes
            head_node_neighbour = []
            row_node_neighbours = []

            # row_length and row_node_dist can be different for different rows
            n_row_nodes = len(numpy.arange(0, row_length[row_id], row_node_dist[row_id])) + 1
            # add row nodes of i-th head node
            for j in range(n_row_nodes):
                row_node_name = "rn-%02d-%02d" %(i, j)
                # only one head node neighbour - the head node of the row
                head_node_neighbour = [head_nodes[head_node_name]]
                row_node_neighbours = []
                # other row nodes
                if (j == 0) and (n_row_nodes == 1):
                    # only one row node, no row node neighbours
                    pass
                elif (j == 0) and (n_row_nodes != 1):
                    # more than one row nodes, but the first one
                    row_node_neighbours.append(row_nodes["rn-%02d-%02d" %(i, j + 1)])
                elif j == n_row_nodes - 1:
                    # for the last node (there would be at least 2 head nodes)
                    row_node_neighbours.append(row_nodes["rn-%02d-%02d" %(i, j - 1)])
                else:
                    # for any other node
                    row_node_neighbours.append(row_nodes["rn-%02d-%02d" %(i, j - 1)])
                    row_node_neighbours.append(row_nodes["rn-%02d-%02d" %(i, j + 1)])
                # add j-th row node of i-th row / head node
                self.graph.add_node(row_nodes[row_node_name],
                                    head_node_neighbour + row_node_neighbours)
            # increment head node counter
            i += 1


class Picker(object):
    """Picker class definition"""
    def __init__(self, name, env, farm, tray_capacity, max_n_trays,
                 picking_rate, transport_rate, loading_time):
        """Create a Picker object

        Keyword arguments:
        name -- name/id of the picker
        env -- simpy.Environment
        farm -- provides the allocation and monitor process as well as a graph.
        """
        self.name = name
        self.env = env
        self.farm = farm
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        self.picking_rate = random.uniform(picking_rate-0.01,
                                           picking_rate+0.01)
        self.transport_rate = transport_rate
        self.max_n_trays = max_n_trays
        self.max_wait_for_allocation = 5    # max time to wait before unloading everything

        self.loading_time = loading_time    # time spent at localStorage
        self.assigned_row = None            # only for picking mode

        self.mode = 0       # 0:free, 1:picking, 2:transporting, 3:finished_job
        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, row_node_dist, last_node_dist, local_storage_node]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_path = []

        self.picking_progress = 0.  # percentage of tray_capacity

        self.transport_progress = 0.

        self.action = self.env.process(self.picking_process())

    def picking_process(self, ):
        """ Picker's picking process
        """
        # 1. picker should report for duty first
        self.farm.picker_report(self.name)
        while True:
            # 2. If the picker is assigned a row,
            #   a. continue picking
            #   b. yield along each row_node_dist
            #   c. check tray_cap,
            #       i. if tray_cap is reached, increment n_trays
            #       ii. if n_trays reach max_n_trays, start the transport_process
            #       ii. if not move to the next row_node in the next iter
            if self.mode == 1:
                # move along each node in the assigned row
                # the picker is at the curr_node
                # move to the node next to curr_node in the row_path
                if self.picking_dir is "forward":
                    curr_node_index = self.row_path.index(self.curr_node)
                    next_node = self.row_path[curr_node_index + 1]
                    if curr_node_index != len(self.row_path) - 2:
                        node_dist = self.curr_row_info[3]
                    else:
                        node_dist = self.curr_row_info[4]

                    time_to_pick = node_dist / self.picking_rate
                    yield self.env.timeout(time_to_pick)

                    # update the picking progress
                    self.picking_progress += self.farm.graph.nodes[self.curr_node].yield_at_node
                    print("%s reached %s from %s at %04.3f" %(self.name, next_node,
                                                              self.curr_node,
                                                              self.env.now))
                    print("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.name,
                                                                                        self.tot_trays,
                                                                                        self.n_trays,
                                                                                        self.picking_progress))
                    self.curr_node = "" + next_node
                    if self.curr_node == self.row_path[-1]:
                        self.picking_dir = "reverse"
                        print("%s changing to reverse along %s at %04.3f" %(self.name,
                                                                            self.curr_row,
                                                                            self.env.now))

                    # if the tray capacity is reached, increment n_trays
                    if self.picking_progress >= self.tray_capacity:
                        self.n_trays += 1
                        self.picking_progress -= self.tray_capacity

                    # if max_n_trays is reached
                    if self.n_trays == self.max_n_trays:
                        # transport to local storage and return
                        trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                       self.local_storage_node)
                        time_to_transport = trans_path_dist / self.transport_rate
                        self.mode = 2
                        print("%s reached %d trays. going to local storage at %04.3f" %(self.name,
                                                                                        self.max_n_trays,
                                                                                        self.env.now))
                        print("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.name,
                                                                                            self.tot_trays,
                                                                                            self.n_trays,
                                                                                            self.picking_progress))
                        yield self.env.process(self.transport_process(time_to_transport, 2))
                        print("%s returned from local storage at %04.3f" %(self.name,
                                                                           self.env.now))

                        # resume picking
                        self.tot_trays += self.max_n_trays
                        self.n_trays -= self.max_n_trays
                        self.mode = 1

                elif self.picking_dir is "reverse":
                    # work with negative indices
                    curr_node_index = self.row_path.index(self.curr_node) - len(self.row_path)
                    next_node = self.row_path[curr_node_index - 1]
                    if curr_node_index != -1:
                        node_dist = self.curr_row_info[3]
                    else:
                        node_dist = self.curr_row_info[4]

                    time_to_pick = node_dist / self.picking_rate
                    yield self.env.timeout(time_to_pick)

                    # update the picking progress
                    self.picking_progress += self.farm.graph.nodes[self.curr_node].yield_at_node
                    self.curr_node = "" + next_node
                    if self.curr_node == self.row_path[0]:
                        # row is finished
                        self.farm.finished_rows[self.curr_row].succeed(value=self.env.now)

                        self.picking_dir = None
                        self.prev_row = "" + self.curr_row
                        self.prev_row_info = [] + self.curr_row_info
                        self.curr_row = None
                        self.row_path = []

                    # check picking progress
                    if self.picking_progress >= self.tray_capacity:
                        self.picking_progress -= self.tray_capacity
                        self.n_trays += 1

                        if self.n_trays == self.max_n_trays:
                            if self.curr_row is None:
                                # transport to local storage
                                trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                               self.local_storage_node)
                                time_to_transport = trans_path_dist / self.transport_rate
                                self.mode = 2
                                print("%s reached %d trays. going to local storage at %04.3f" %(self.name,
                                                                                                self.max_n_trays,
                                                                                                self.env.now))
                                yield self.env.process(self.transport_process(time_to_transport, 1))
                                # finished the allocated row and transported all berries
                                # now at local_storage_node
                                # no current allocation - change mode to zero
                                self.tot_trays += self.max_n_trays
                                self.n_trays -= self.max_n_trays
                                self.curr_node = "" + self.local_storage_node
                                self.mode = 0
                            else:
                                # transport to local storage and return
                                trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                               self.local_storage_node)
                                time_to_transport = trans_path_dist / self.transport_rate
                                self.mode = 2
                                print("%s reached %d trays. going to local storage at %04.3f" %(self.name,
                                                                                                self.max_n_trays,
                                                                                                self.env.now))
                                yield self.env.process(self.transport_process(time_to_transport, 2))
                                print("%s returned from local storage at %04.3f" %(self.name,
                                                                                   self.env.now))

                                # resume picking
                                self.tot_trays += self.max_n_trays
                                self.n_trays -= self.max_n_trays
                                self.mode = 1
                    else:
                        # picking_progress != tray_capacity : go to not allocated mode
                        if self.curr_row is None:
                            # no current allcoations
                            self.mode = 0

            # 3. If in mode free, check if there is any new assignments
            #       If there is no new assignment and no rows left to be assigned, finish picking
            #       If there is a new assignment
            #           a. Move to the start node of the path
            #           b. Get the path from the current loc to start_node of new row
            #           c. Move at transport_rate to start_node
            #           d. Continue/Start picking along the new row by changing mode to picking
            elif self.mode == 0:
                row_id = self.farm.curr_picker_allocations[self.name]

                if (row_id is None) and (len(self.farm.unallocated_rows) == 0):
                    if self.curr_node == self.local_storage_node:
                        # at local storage after unloading max_n_trays
                        # if there is anything left, unload those and leave the picking process
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            self.env.process(self.transport_process(0, 1))
                            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
                            self.n_trays = 0
                            self.picking_progress = 0
                        self.mode = 3
                    elif self.curr_node is not None:
                        # not the first allocation, meaning the picker is at some node already
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                           self.local_storage_node)
                            time_to_transport = trans_path_dist / self.transport_rate
                            self.mode = 2
                            self.env.process(self.transport_process(time_to_transport, 1))
                            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
                            self.n_trays = 0
                            self.picking_progress = 0

                        self.mode = 3
                    # finish the picking process
                    print("%s finishing picking process. all rows are assigned" %(self.name))
                    break

                if row_id is not None: # if there is a row assigned to the picker
                    self.curr_row = row_id
                    self.curr_row_info = self.farm.row_info[self.curr_row]

                    # set local storage as curr_node if never assigned before
                    if self.curr_node is None:
                        self.curr_node = self.curr_row_info[5]
                        self.local_storage_node = self.curr_row_info[5]

                    print("%s is moving to the start of %s at %04.3f" %(self.name,
                                                                        self.curr_row,
                                                                        self.env.now))
                    # transport to the start_node of the row
                    trans_path, trans_path_dist = self.farm.graph.get_path_details(self.curr_node,
                                                                                   self.curr_row_info[1])
                    time_to_transport = trans_path_dist / self.transport_rate
                    yield self.env.timeout(time_to_transport)

                    # picker moved to the start_node of the row (yield above)
                    self.curr_node = self.curr_row_info[1]
                    self.row_path = self.farm.graph.get_path(self.curr_node, self.curr_row_info[2])

                    print("%s started forward picking on %s at %04.3f" %(self.name, row_id, self.env.now))
                    # change current mode to picking
                    self.mode = 1 # picking mode

                    self.picking_dir = "forward"

            yield self.env.timeout(1)

    def transport_process(self, time_to_transport, n_times):
        """Picker's transportation process
        """
        # This node should ideally implement picker's transportation to the local storage
        # and return if needed.
        # Only a timeout implementation is done now.
        #   1. move along the path (yield timeout(time_to_travel_path))
        yield self.env.timeout(time_to_transport)
        #   2. request for the local storage access
        #   3. wait further for unloading (yield timeout(loading_time))
        print("%s requesting for local_storage resource at %0.3f" %(self.name, self.env.now))
        with self.farm.local_storages[self.local_storage_node].request() as req:
            yield req
            print("%s got access to local_storage resource at %0.3f" %(self.name, self.env.now))
            yield self.env.timeout(self.loading_time)
            print("%s spent %0.3f for unloading at the local_staorage" %(self.name, self.loading_time))
        # if needed, return to previous node
        if n_times == 2:
            yield self.env.timeout(time_to_transport)


if __name__ == "__main__":
    random.seed(RANDOM_SEED)
    env = simpy.Environment()
#    env = simpy.RealtimeEnvironment(initial_time=0, factor=1.0, strict=False)
    # assuming the following:
    # 1. only one head lane
    # 2. a picker won't have to wait longer than loadingTime
    local_storages = [simpy.Resource(env, capacity=N_PICKERS)]
    farm = Farm("RAS-Berry", env)
    farm.init_graph_fork(N_ROWS, ROW_NODE_DIST, ROW_LENGTH, ROW_SPACING, YIELD_PER_NODE, local_storages)
#    farm.graph.print_nodes()
    env.process(farm.scheduler_monitor())

    pickers = []
    for i in range(N_PICKERS):
        pickers.append(Picker("picker-%02d" %(i), env, farm, TRAY_CAPACITY,
                              MAX_N_TRAYS, PICKING_RATE, TRANSPORT_RATE, LOADING_TIME))

    env.run(until=60)

    # farm details
    print("-----------------\n----%s----\n-----------------" %(farm.name))
    print("n_pickers: %d" %(len(farm.pickers_reported)))
    print("n_rows: %d" %(farm.n_rows))
    tot_yield = 0.
    for row_id in farm.rows:
        print("  --%s--" %(row_id))
        row_length = farm.graph.nodes[farm.row_info[row_id][2]].y
        node_dist = farm.row_info[row_id][3]
        print("  row_length: %0.3f m" %(row_length))
        print("  node_dist: %0.3f m" %(node_dist))
        row_yield = 0.
        n_row_nodes = len(numpy.arange(0, row_length, node_dist)) + 1
        for i in range(n_row_nodes):
            if (i == 0) or (i == n_row_nodes - 1):
                row_yield += farm.graph.nodes[farm.row_node_names[row_id][i]].yield_at_node
            else:
                row_yield += 2 * farm.graph.nodes[farm.row_node_names[row_id][i]].yield_at_node
        print("  row_yield: %0.3f g" %(row_yield))
        tot_yield += row_yield
    print("tot_yield: %0.3f g" %(tot_yield))
    print("\n")

    # picker details
    for i in range(N_PICKERS):
        print("----%s----\n-----------------" %(pickers[i].name))
        print("picking_rate: %0.3f m/s" %(pickers[i].picking_rate))
        print("transport_rate: %0.3f m/s" %(pickers[i].transport_rate))
        print("tray_capacity: %d g" %(pickers[i].tray_capacity))
        print("max_n_trays: %d" %(pickers[i].max_n_trays))
        print("rows allocated: ", farm.picker_allocations[pickers[i].name])
        for row_id in farm.picker_allocations[pickers[i].name]:
            print("  %s allocation time: %0.3f" %(row_id, farm.allocation_time[row_id]))
            print("  %s completion time: %0.3f" %(row_id, farm.row_finish_time[row_id]))
        print("tot_trays: %0.3f (%0.3f g)" %(pickers[i].tot_trays, pickers[i].tot_trays * pickers[i].tray_capacity))
        print("-----------------\n")
