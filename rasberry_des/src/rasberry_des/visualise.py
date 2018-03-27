#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import matplotlib.pyplot
import matplotlib.animation
import rospy
import geometry_msgs.msg
import rasberry_des.msg


class Visualise_Agents(object):
    """A class to animate agent locations in matplotlib"""
    def __init__(self, farm, topo_graph, picker_ids, robot_ids, detailed=False):
        """initialise the Visualise_Agents class

        Keyword arguments:

        farm -- rasberry_des.farm.Farm object
        pickers == list of rasberry_des.picker.Picker objects
        """
        self.n_pickers = len(picker_ids)
        self.picker_ids = picker_ids
        self.robot_ids = robot_ids
        self.n_robots = len(robot_ids)
        self.farm = farm
        self.detailed = detailed
        self.graph = topo_graph

        self.fig = matplotlib.pyplot.figure()
        self.ax = self.fig.add_subplot(111)
        self.font = {'family': 'serif', 'color':  'darkred', 'weight': 'normal', 'size': 8,}

        # publishers / subscribers
        # picker related
        self.picker_pose_subs = {}
        self.init_picker_pose_subs()
        self.picker_x = {picker_id:0. for picker_id in self.picker_ids}
        self.picker_y = {picker_id:0. for picker_id in self.picker_ids}
        self.picker_position_lines = {}
        self.picker_position_texts = {}
        # robot related
        self.robot_pose_subs = {}
        self.init_robot_pose_subs()
        self.robot_x = {robot_id:0. for robot_id in self.robot_ids}
        self.robot_y = {robot_id:0. for robot_id in self.robot_ids}
        self.robot_position_lines = {}
        self.robot_position_texts = {}

        if self.detailed:
            self.picker_status_subs = {}
            self.picker_picking_progress = {picker_id:0. for picker_id in self.picker_ids}
            self.picker_n_trays = {picker_id:0 for picker_id in self.picker_ids}
            self.picker_tot_trays = {picker_id:0 for picker_id in self.picker_ids}
            self.picker_n_rows = {picker_id:0 for picker_id in self.picker_ids}
            self.picker_mode = {picker_id:0 for picker_id in self.picker_ids}

            self.robot_status_subs = {}
            self.init_robot_status_subs()
            self.robot_n_empty_trays = {robot_id:0 for robot_id in self.robot_ids}
            self.robot_n_full_trays = {robot_id:0 for robot_id in self.robot_ids}
            self.robot_tot_trays = {robot_id:0. for robot_id in self.robot_ids}
            self.robot_mode= {robot_id:0 for robot_id in self.robot_ids}

        self.init_plot()
#        self.ani = matplotlib.animation.FuncAnimation(self.fig, func=self.plot_update, blit=False, interval=100)
        # show the plot
        matplotlib.pyplot.show(block=False)

    def init_plot(self, ):
        """Initialise the plot frame"""
        farm_rows_x, farm_rows_y = [], []
        nav_rows_x, nav_rows_y = [], []
        nav_row_nodes_x, nav_row_nodes_y = [], []
        head_lane_x, head_lane_y = [], []
        head_nodes_x, head_nodes_y = [], []
        local_storage_x, local_storage_y = [], []
        local_storage_nodes = []
        for i in range(self.farm.n_topo_nav_rows):
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
            if (i == 0) or (i == self.farm.n_topo_nav_rows - 1):
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
                if i != self.farm.n_topo_nav_rows - 1:
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
        # local storages
        self.ax.plot(local_storage_x, local_storage_y, color="black", marker="s", markersize=12,
                     markeredgecolor="r", linestyle="none")
        # dynamic objects - pickers and robots
        # pickers
        for picker_id in self.picker_ids:
            self.picker_position_lines[picker_id] = self.ax.plot(self.picker_x[picker_id],
                                                                 self.picker_y[picker_id],
                                                                 color="blue", marker="8",
                                                                 markersize=20,
                                                                 markeredgecolor="r",
                                                                 linestyle="none")[0]
            if self.detailed:
                self.picker_position_texts[picker_id] = self.ax.text(self.picker_x[picker_id] -0.75,
                                                                     self.picker_y[picker_id] + 0.3,
                                                                     "P_%s\n%0.2f\n%d\n%d\n%d\n%d" %(picker_id[-2:],
                                                                                                 self.picker_picking_progress[picker_id],
                                                                                                 self.picker_n_trays[picker_id],
                                                                                                 self.picker_tot_trays[picker_id],
                                                                                                 self.picker_n_rows[picker_id],
                                                                                                 self.picker_mode[picker_id]),
                                                                     fontdict=self.font)
            else:
                self.picker_position_texts[picker_id] = self.ax.text(self.picker_x[picker_id] -0.75,
                                                                     self.picker_y[picker_id] + 0.3,
                                                                     "P_%s" %(picker_id[-2:]), fontdict=self.font)
        # robots
        for robot_id in self.robot_ids:
            self.robot_position_lines[robot_id] = self.ax.plot(self.robot_x[robot_id],
                                                                 self.robot_y[robot_id],
                                                                 color="green", marker="*",
                                                                 markersize=20,
                                                                 markeredgecolor="r",
                                                                 linestyle="none")[0]
            if self.detailed:
                self.robot_position_texts[robot_id] = self.ax.text(self.robot_x[robot_id] -0.75,
                                                                     self.robot_y[robot_id] + 0.3,
                                                                     "P_%s\n%0.2f\n%d\n%d\n%d\n%d" %(robot_id[-2:],
                                                                                                 self.robot_n_empty_trays[robot_id],
                                                                                                 self.robot_n_full_trays[robot_id],
                                                                                                 self.robot_tot_trays[robot_id],
                                                                                                 self.robot_mode[robot_id]),
                                                                     fontdict=self.font)
            else:
                self.robot_position_texts[robot_id] = self.ax.text(self.robot_x[robot_id] -0.75,
                                                                     self.robot_y[robot_id] + 0.3,
                                                                     "R_%s" %(robot_id[-2:]), fontdict=self.font)

    def init_picker_pose_subs(self, ):
        ns = rospy.get_namespace()
        for picker_id in self.picker_ids:
            self.picker_pose_subs[picker_id] = rospy.Subscriber(ns + "%s/pose"%(picker_id),
                                                                geometry_msgs.msg.Pose,
                                                                self.update_picker_position,
                                                                callback_args=picker_id)

    def init_picker_status_subs(self, ):
        ns = rospy.get_namespace()
        for picker_id in self.picker_ids:
            self.picker_pose_subs[picker_id] = rospy.Subscriber(ns + "%s/status"%(picker_id),
                                                                rasberry_des.msg.Picker_Status,
                                                                self.update_picker_status,
                                                                callback_args=picker_id)

    def update_picker_position(self, msg, picker_id):
        self.picker_x[picker_id] = msg.position.x
        self.picker_y[picker_id] = msg.position.y

    def update_picker_status(self, msg, picker_id):
        self.picker_picking_progress[picker_id] = msg.picking_progress
        self.picker_n_trays[picker_id] = msg.n_trays
        self.picker_tot_trays[picker_id] = msg.tot_trays
        self.picker_n_rows[picker_id] = msg.n_rows
        self.picker_modes[picker_id] = msg.mode

    def init_robot_pose_subs(self, ):
        ns = rospy.get_namespace()
        for robot_id in self.robot_ids:
            self.robot_pose_subs[robot_id] = rospy.Subscriber(ns + "%s/pose"%(robot_id),
                                                                geometry_msgs.msg.Pose,
                                                                self.update_robot_position,
                                                                callback_args=robot_id)

    def init_robot_status_subs(self, ):
        ns = rospy.get_namespace()
        for robot_id in self.robot_ids:
            self.robot_pose_subs[robot_id] = rospy.Subscriber(ns + "%s/status"%(robot_id),
                                                                rasberry_des.msg.Robot_Status,
                                                                self.update_robot_status,
                                                                callback_args=robot_id)

    def update_robot_position(self, msg, robot_id):
        self.robot_x[robot_id] = msg.position.x
        self.robot_y[robot_id] = msg.position.y

    def update_robot_status(self, msg, robot_id):
        self.robot_n_empty_trays[robot_id] = msg.n_empty_trays
        self.robot_n_full_trays[robot_id] = msg.n_full_trays
        self.robot_tot_trays[robot_id] = msg.tot_trays
        self.robot_mode[robot_id] = msg.mode

    def plot_update(self, *args):
        for picker_id in self.picker_ids:
            self.picker_position_lines[picker_id].set_data(self.picker_x[picker_id],
                                                           self.picker_y[picker_id])
            if self.detailed:
                self.picker_position_texts[picker_id].set_position(self.ax.text(self.picker_x[picker_id] -0.75,
                                                                                self.picker_y[picker_id] + 0.3,
                                                                                "P_%s\n%0.2f\n%d\n%d\n%d" %(picker_id[-2:],
                                                                                                            self.picker_picking_progress[picker_id],
                                                                                                            self.picker_n_trays[picker_id],
                                                                                                            self.picker_tot_trays[picker_id],
                                                                                                            self.picker_n_rows[picker_id],
                                                                                                            self.picker_mode[picker_id]),
                                                                                fontdict=self.font))
            else:
                self.picker_position_texts[picker_id].set_position((self.picker_x[picker_id] -0.75,
                                                                    self.picker_y[picker_id] + 0.3))

        for robot_id in self.robot_ids:
            self.robot_position_lines[robot_id].set_data(self.robot_x[robot_id],
                                                           self.robot_y[robot_id])
            if self.detailed:
                self.robot_position_texts[robot_id].set_position(self.ax.text(self.robot_x[robot_id] -0.75,
                                                                                self.robot_y[robot_id] + 0.3,
                                                                                "P_%s\n%0.2f\n%d\n%d\n%d" %(robot_id[-2:],
                                                                                                            self.robot_n_empty_trays[robot_id],
                                                                                                            self.robot_n_full_trays[robot_id],
                                                                                                            self.robot_tot_trays[robot_id],
                                                                                                            self.robot_mode[robot_id],),
                                                                                fontdict=self.font))
            else:
                self.robot_position_texts[robot_id].set_position((self.robot_x[robot_id] -0.75,
                                                                    self.robot_y[robot_id] + 0.3))

        self.fig.canvas.draw()



