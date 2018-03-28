#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import matplotlib.pyplot
import matplotlib.animation

class Visualise_Agents(object):
    """A class to animate agent locations in matplotlib"""
    def __init__(self, topo_graph, robots, pickers):
        """initialise the Visualise_Agents class

        Keyword arguments:

        farm -- rasberry_des.farm.Farm object
        pickers == list of rasberry_des.picker.Picker objects
        """
        self.pickers = pickers
        self.n_pickers = len(self.pickers)
        self.robots = robots
        self.n_robots = len(self.robots)
        self.graph = topo_graph

        self.fig = matplotlib.pyplot.figure()
        self.ax = self.fig.add_subplot(111)
        self.font = {'family': 'serif', 'color':  'darkred', 'weight': 'normal', 'size': 8,}

        self.static_lines = []
        self.picker_position_lines = {}
        self.picker_status_texts = {}
        self.robot_position_lines = {}
        self.robot_status_texts = {}

        self.init_plot()
#        self.ani = matplotlib.animation.FuncAnimation(fig=self.fig, init_func=self.init_plot, func=self.update_plot)

        # show the plot
        matplotlib.pyplot.show(block=False)

    def close_plot(self, ):
        matplotlib.pyplot.close(self.fig)

    def init_plot(self, ):
        """Initialise the plot frame"""
        farm_rows_x, farm_rows_y = [], []
        nav_rows_x, nav_rows_y = [], []
        nav_row_nodes_x, nav_row_nodes_y = [], []
        head_lane_x, head_lane_y = [], []
        head_nodes_x, head_nodes_y = [], []
        local_storage_x, local_storage_y = [], []
        local_storage_nodes = []
        for i in range(self.graph.n_topo_nav_rows):
            row_id = self.graph.row_ids[i]
            head_node = self.graph.get_node(self.graph.head_nodes[row_id])
            head_nodes_x.append(head_node.pose.position.x)
            head_nodes_y.append(head_node.pose.position.y)
            for j in range(len(self.graph.row_nodes[row_id])):
                curr_node = self.graph.get_node(self.graph.row_nodes[row_id][j])
                if j == 0:
                    start_node = curr_node
                elif j == len(self.graph.row_nodes[row_id]) - 1:
                    last_node = curr_node
                nav_row_nodes_x.append(curr_node.pose.position.x)
                nav_row_nodes_y.append(curr_node.pose.position.y)

            # from head_node to last_row_node of the row
            nav_rows_x.append((head_node.pose.position.x, last_node.pose.position.x))
            nav_rows_y.append((head_node.pose.position.y, last_node.pose.position.y))

            # head lane
            if (i == 0) or (i == self.graph.n_topo_nav_rows - 1):
                head_lane_x.append(head_node.pose.position.x)
                head_lane_y.append(head_node.pose.position.y)

            if self.graph.half_rows:
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
                if i != self.graph.n_topo_nav_rows - 1:
                    farm_rows_x.append((start_node_x, last_node_x))
                    farm_rows_y.append((start_node.pose.position.y, last_node.pose.position.y))

            if self.graph.local_storage_nodes[row_id] not in local_storage_nodes:
                local_storage_nodes.append(self.graph.local_storage_nodes[row_id])
                node_obj = self.graph.get_node(local_storage_nodes[-1])
                local_storage_x.append(node_obj.pose.position.x)
                local_storage_y.append(node_obj.pose.position.y)

        # TODO: assuming there is at least two rows are present
        min_x = min(min(nav_rows_x[0]), min(farm_rows_x[0]))
        max_x = max(max(nav_rows_x[-1]), max(farm_rows_x[-1]))
        min_y = min(min(nav_rows_y[0]), min(farm_rows_y[0]))
        max_y = max(max(nav_rows_y[-1]), max(farm_rows_y[-1]))
        # limits of the axes
        self.ax.set_xlim(min_x - 1, max_x + 1)
        self.ax.set_ylim(min_y - 1, max_y + 1)
        self.fig.set_figheight(max_y - min_y + 2)
        self.fig.set_figwidth(max_x - min_x + 2)

        # static objects - nodes
        # nav_rows
        for i in range(len(nav_rows_x)):
            self.static_lines.append(self.ax.plot(nav_rows_x[i], nav_rows_y[i], color="black", linewidth=2)[0])
        # farm_rows
        for i in range(len(farm_rows_x)):
            self.static_lines.append(self.ax.plot(farm_rows_x[i], farm_rows_y[i], color="green", linewidth=8)[0])
        # head lane
        self.static_lines.append(self.ax.plot(head_lane_x, head_lane_y, color="black", linewidth=2)[0])
        # nav_row_nodes
        self.static_lines.append(self.ax.plot(nav_row_nodes_x, nav_row_nodes_y, color="black", marker="o", linestyle="none")[0])
        # head_lane_nodes
        self.static_lines.append(self.ax.plot(head_nodes_x, head_nodes_y, color="black", marker="o", linestyle="none")[0])
        # local storages
        self.static_lines.append(self.ax.plot(local_storage_x, local_storage_y, color="black", marker="s", markersize=12,
                     markeredgecolor="r", linestyle="none")[0])
        # dynamic objects - pickers and robots
        # pickers
        for picker in self.pickers:
            picker_id = picker.picker_id
            if picker.curr_node is not None:
                curr_node_obj = self.graph.get_node(picker.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.picker_position_lines[picker_id] = self.ax.plot(x, y,
                                                                 color="blue", marker="8",
                                                                 markersize=20,
                                                                 markeredgecolor="r",
                                                                 linestyle="none")[0]
            self.picker_status_texts[picker_id] = self.ax.text(x -0.75, y + 0.3,
                                                               "P_%s" %(picker_id[-2:]), fontdict=self.font)
        # robots
        for robot in self.robots:
            robot_id = robot.robot_id
            if robot.curr_node is not None:
                curr_node_obj = self.graph.get_node(robot.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.robot_position_lines[robot_id] = self.ax.plot(x, y,
                                                                 color="green", marker="*",
                                                                 markersize=20,
                                                                 markeredgecolor="r",
                                                                 linestyle="none")[0]
            self.robot_status_texts[robot_id] = self.ax.text(x - 0.75, y + 0.3,
                                                                 "R_%s" %(robot_id[-2:]), fontdict=self.font)

    def update_plot(self, *args):
        for picker in self.pickers:
            picker_id = picker.picker_id
            if picker.curr_node is not None:
                curr_node_obj = self.graph.get_node(picker.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.

            self.picker_position_lines[picker_id].set_data(x, y)
            self.picker_status_texts[picker_id].set_position((x - 0.75, y + 0.3))

        for robot in self.robots:
            robot_id = robot.robot_id
            if robot.curr_node is not None:
                curr_node_obj = self.graph.get_node(robot.curr_node)
                x = curr_node_obj.pose.position.x
                y = curr_node_obj.pose.position.y
            else:
                x = y = 0.
            self.robot_position_lines[robot_id].set_data(x, y)
            self.robot_status_texts[robot_id].set_position((x - 0.75, y + 0.3))

        self.fig.canvas.draw()
