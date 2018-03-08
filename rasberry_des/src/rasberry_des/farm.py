#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import simpy
import numpy
import topo


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

        self.row_ids = []              # can be a row name / id
        self.n_farm_rows = 0
        self.half_rows = False
        self.n_topo_nav_rows = 0

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

    def init_graph(self, f_name=None):
        """Generate a graph from the given file"""
        # TODO: fill here with code to read the file and generate the graph
        pass

    def init_graph_fork(self, n_farm_rows, half_rows, n_topo_nav_rows,
                        _head_row_node_dist, _head_node_y,
                        _row_node_dist, _row_length, _row_spacing,
                        _yield_per_node, local_storages):
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
        # words in node names are separated with -
        # words in tags (row_ids) are separated with _
        # words in edge_ids are separated with _
        self.n_farm_rows = n_farm_rows
        self.half_rows = half_rows
        self.n_topo_nav_rows = n_topo_nav_rows

        self.row_ids = ["row_%02d" %(i) for i in range(self.n_topo_nav_rows)]   # can be a row name / id
        self.unallocated_rows = [] + self.row_ids

        self.finished_rows = {row_id:simpy.Event(self.env) for row_id in self.row_ids}
        self.row_finish_time = {row_id:None for row_id in self.row_ids}

        self.allocations = {row_id:None for row_id in self.row_ids}
        self.allocation_time = {row_id:None for row_id in self.row_ids}

        if _head_row_node_dist.__class__ == list:
            if len(_head_row_node_dist) == n_topo_nav_rows:
                head_row_node_dist = {self.row_ids[i]:_head_row_node_dist[i] for i in range(n_topo_nav_rows)}
            elif len(_head_row_node_dist) == 1:
                head_row_node_dist = {self.row_ids[i]:_head_row_node_dist[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("head_row_node_dist must be list of length %d or 1" %(n_topo_nav_rows))

        if _head_node_y.__class__ == list:
            if len(_head_node_y) == n_topo_nav_rows:
                head_node_y = {self.row_ids[i]:_head_node_y[i] for i in range(n_topo_nav_rows)}
            elif len(_head_node_y) == 1:
                head_node_y = {self.row_ids[i]:_head_node_y[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("head_node_y must be list of length %d or 1" %(n_topo_nav_rows))

        if _row_node_dist.__class__ == list:
            if len(_row_node_dist) == n_topo_nav_rows:
                row_node_dist = {self.row_ids[i]:_row_node_dist[i] for i in range(n_topo_nav_rows)}
            elif len(_row_node_dist) == 1:
                row_node_dist = {self.row_ids[i]:_row_node_dist[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("row_node_dist must be list of length %d or 1" %(n_topo_nav_rows))

        if _row_length.__class__ == list:
            if len(_row_length) == n_topo_nav_rows:
                row_length = {self.row_ids[i]:_row_length[i] for i in range(n_topo_nav_rows)}
            elif len(_row_length) == 1:
                row_length = {self.row_ids[i]:_row_length[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("row_length must be list of length %d or 1" %(n_topo_nav_rows))

        if _row_spacing.__class__ == list:
            if len(_row_spacing) == n_topo_nav_rows:
                row_spacing = {self.row_ids[i]:_row_spacing[i] for i in range(n_topo_nav_rows)}
            elif len(_row_spacing) == 1:
                row_spacing = {self.row_ids[i]:_row_spacing[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("row_spacing must be list of length %d or 1" %(n_topo_nav_rows))

        if _yield_per_node.__class__ == list:
            if len(_yield_per_node) == n_topo_nav_rows:
                yield_per_node = {self.row_ids[i]:_yield_per_node[i] for i in range(n_topo_nav_rows)}
            elif len(_yield_per_node) == 1:
                yield_per_node = {self.row_ids[i]:_yield_per_node[0] for i in range(n_topo_nav_rows)}
        else:
            raise TypeError("yield_per_node must be list of length %d or 1" %(n_topo_nav_rows))

        last_node_dist = {row_id:0. for row_id in self.row_ids}

        self.graph = topo.BiGraph()

        row_nodes = {}
        head_nodes = {}

        # TODO: There can be more than one local storage nodes
        # As there are no guidelines about this at this stage, only one storage node is added
        # It is assumed that there is only one simpy.Resource object in local_storages
        # Place it at int(n_topo_nav_rows/2)
        self.local_storage_nodes.append("hn-%02d" %(int(round(self.n_topo_nav_rows / 2))))
        self.local_storages[self.local_storage_nodes[0]] = local_storages[0]

        # 1. create the nodes - head_nodes and row_nodes
        i = 0   # head node counter: one head node per row
        x = 0.
        for row_id in self.row_ids:
            # All head nodes are Node objects with zero yield
            # All row nodes are Node objects with a yield_at_node
            x += row_spacing[row_id] / 2.
            y = head_node_y[row_id]

            head_node_name = "hn-%02d" %(i)
            head_nodes[head_node_name] = topo.Node(head_node_name, x, y)

            self.head_node_names.append(head_node_name)
            self.row_node_names[row_id] = []

            # row length can be different for different rows
            if row_length[row_id] > 0.:
                # 1 is for the end node, which is not produced in numpy.arange
                n_row_nodes = len(numpy.arange(0, row_length[row_id], row_node_dist[row_id])) + 1
            else:
                n_row_nodes = 0

            for j in range(n_row_nodes):
                if j == 0:
                    y += head_row_node_dist[row_id]
                else:
                    # row_length may not be an exact multiple of row_node_dist
                    y += row_node_dist[row_id] if j != n_row_nodes - 1 else row_length[row_id] - (j - 1) * row_node_dist[row_id]

                # yield from each row node to next row node
                if j != n_row_nodes - 1:
                    yield_at_node = numpy.random.logistic(yield_per_node[row_id])
                else:
                    # between the last two nodes, the distance could be smaller than node_dist
                    last_node_dist[row_id] = row_length[row_id] - (row_node_dist[row_id] * (j - 1))
                    yield_at_node = numpy.random.logistic((yield_per_node[row_id] * last_node_dist[row_id]) / row_node_dist[row_id])

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
        for row_id in self.row_ids:
            # neighbours of head nodes
            head_node_neighbours = []
            row_node_neighbour = []

            head_node_name = "hn-%02d" %(i)
            # neighbouring head nodes
            head_node_neighbours = []
            if (i == 0) and (self.n_topo_nav_rows == 1):
                # if the only head node, there are no head node neighbours
                pass
            elif (i == 0) and (self.n_topo_nav_rows != 1):
                # more than one head node, add the next head node
                head_node_neighbours.append(head_nodes["hn-%02d" %(i + 1)])
            elif i == self.n_topo_nav_rows - 1:
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
