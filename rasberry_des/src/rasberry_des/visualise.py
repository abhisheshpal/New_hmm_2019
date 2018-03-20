#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


from matplotlib import pyplot
import topological_navigation.tmap_utils


class Visualise_Agents(object):
    """A class to animate agent locations in matplotlib"""
    def __init__(self, farm, pickers):
        """initialise the Visualise_Agents class

        Keyword arguments:

        farm -- rasberry_des.farm.Farm object
        pickers == list of rasberry_des.picker.Picker objects
        """
        self.n_pickers = len(pickers)
        self.pickers = pickers
        self.farm = farm
        self.fig = pyplot.figure()
        self.ax = self.fig.add_subplot(111)
        self.init_plot()
        # show the plot
        pyplot.show()

    def init_plot(self, ):
        """Initialise the plot frame"""
        farm_rows_x, farm_rows_y = [], []
        nav_rows_x, nav_rows_y = [], []
        nav_row_nodes_x, nav_row_nodes_y = [], []
        head_lane_x, head_lane_y = [], []
        head_nodes_x, head_nodes_y = [], []
        for i in range(self.farm.n_topo_nav_rows):
            row_id = self.farm.row_ids[i]
            head_node = topological_navigation.tmap_utils.get_node(self.farm.graph.topo_map, self.farm.graph.head_nodes[row_id])
            head_nodes_x.append(head_node.pose.position.x)
            head_nodes_y.append(head_node.pose.position.y)
            for j in range(len(self.farm.graph.row_nodes[row_id])):
                curr_node = topological_navigation.tmap_utils.get_node(self.farm.graph.topo_map, self.farm.graph.row_nodes[row_id][j])
                if j == 0:
                    start_node = curr_node
                elif j == len(self.farm.graph.row_nodes[row_id]) - 1:
                    last_node = curr_node
                nav_row_nodes_x.append(curr_node.pose.position.x)
                nav_row_nodes_y.append(curr_node.pose.position.y)

            # from head_node to last_row_node of the row
            nav_rows_x.append((head_node.pose.position.x, last_node.pose.position.x))
            nav_rows_y.append((head_node.pose.position.y, last_node.pose.position.y))

            # head lane
            if (i == 0) or (i == self.farm.n_topo_nav_rows - 1):
                head_lane_x.append(head_node.pose.position.x)
                head_lane_y.append(head_node.pose.position.y)

            if self.farm.half_rows:
                if i == 0:
                    farm_rows_x.append((0., 0.))
                    farm_rows_y.append((start_node.pose.position.y, last_node.pose.position.y))

                # from start_row_node to last_row_node
                start_node_x = 2 * start_node.pose.position.x - farm_rows_x[-1][0]
                last_node_x = 2 * last_node.pose.position.x - farm_rows_x[-1][1]
                farm_rows_x.append((start_node_x, last_node_x))
                farm_rows_y.append((start_node.pose.position.y, last_node.pose.position.y))

            else:
                # from start_row_node to last_row_node
                if i == 0:
                    start_node_x = 2 * start_node.pose.position.x
                    last_node_x = 2 * last_node.pose.position.x
                else:
                    start_node_x = 2 * start_node.pose.position.x - farm_rows_x[-1][0]
                    last_node_x = 2 * last_node.pose.position.x - farm_rows_x[-1][1]
                if i != self.farm.n_topo_nav_rows - 1:
                    farm_rows_x.append((start_node_x, last_node_x))
                    farm_rows_y.append((start_node.pose.position.y, last_node.pose.position.y))

        # TODO: assuming there is at least two rows are present
        min_x = min(min(nav_rows_x[0]), min(farm_rows_x[0]))
        max_x = max(max(nav_rows_x[-1]), max(farm_rows_x[-1]))
        min_y = min(min(nav_rows_y[0]), min(farm_rows_y[0]))
        max_y = max(max(nav_rows_y[-1]), max(farm_rows_y[-1]))
        # limits of the axes
        self.ax.set_xlim(min_x - 1, max_x + 1)
        self.ax.set_ylim(min_y - 1, max_y + 1)
        # static objects - nodes
        # nav_rows
        for i in range(len(nav_rows_x)):
            self.ax.plot(nav_rows_x[i], nav_rows_y[i], color="black", linewidth=2)
        # farm_rows
        for i in range(len(farm_rows_x)):
            self.ax.plot(farm_rows_x[i], farm_rows_y[i], color="green", linewidth=8)
        # head lane
        self.ax.plot(head_lane_x, head_lane_y, color="black", linewidth=2)
        # nav_row_nodes
        self.ax.plot(nav_row_nodes_x, nav_row_nodes_y, color="black", marker="o", linestyle="none")
        # head_lane_nodes
        self.ax.plot(head_nodes_x, head_nodes_y, color="black", marker="o", linestyle="none")
        # dynamic objects - pickers and robots
        # pickers
