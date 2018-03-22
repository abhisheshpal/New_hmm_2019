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
    def __init__(self, farm, pickers, detailed=False):
        """initialise the Visualise_Agents class

        Keyword arguments:

        farm -- rasberry_des.farm.Farm object
        pickers == list of rasberry_des.picker.Picker objects
        """
        self.n_pickers = len(pickers)
        self.pickers = pickers
        self.farm = farm
        self.detailed = detailed

        self.fig = matplotlib.pyplot.figure()
        self.ax = self.fig.add_subplot(111)
        self.font = {'family': 'serif', 'color':  'darkred', 'weight': 'normal', 'size': 8,}

        # dynamic object related
        self.picker_pose_subs = []
        self.set_picker_pose_subs()
        self.picker_x = {self.pickers[i].picker_id:0. for i in range(self.n_pickers)}
        self.picker_y = {self.pickers[i].picker_id:0. for i in range(self.n_pickers)}
        self.picker_position_lines = []
        self.picker_position_texts = []

        if self.detailed:
            self.picker_status_subs = []
            self.set_picker_status_subs()
            self.picker_picking_progress = {self.pickers[i].picker_id:0. for i in range(self.n_pickers)}
            self.picker_n_trays = {self.pickers[i].picker_id:0 for i in range(self.n_pickers)}
            self.picker_tot_trays = {self.pickers[i].picker_id:0 for i in range(self.n_pickers)}
            self.picker_n_rows = {self.pickers[i].picker_id:0 for i in range(self.n_pickers)}

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
            row_id = self.farm.row_ids[i]
            head_node = self.farm.graph.get_node(self.farm.graph.head_nodes[row_id])
            head_nodes_x.append(head_node.pose.position.x)
            head_nodes_y.append(head_node.pose.position.y)
            for j in range(len(self.farm.graph.row_nodes[row_id])):
                curr_node = self.farm.graph.get_node(self.farm.graph.row_nodes[row_id][j])
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

            if self.farm.graph.local_storage_nodes[row_id] not in local_storage_nodes:
                local_storage_nodes.append(self.farm.graph.local_storage_nodes[row_id])
                node_obj = self.farm.graph.get_node(local_storage_nodes[-1])
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
        for i in range(self.n_pickers):
            self.picker_position_lines.append(self.ax.plot(self.picker_x[self.pickers[i].picker_id],
                                                           self.picker_y[self.pickers[i].picker_id],
                                                           color="blue", marker="8", markersize=20,
                                                           markeredgecolor="r", linestyle="none")[0])
            if self.detailed:
                self.picker_position_texts.append(self.ax.text(self.picker_x[self.pickers[i].picker_id] -0.75,
                                                               self.picker_y[self.pickers[i].picker_id] + 0.3,
                                                               "P_%s\n%0.2f\n%d\n%d\n%d" %(self.pickers[i].picker_id[-2:],
                                                                                           self.picker_picking_progress[self.pickers[i].picker_id],
                                                                                           self.picker_n_trays[self.pickers[i].picker_id],
                                                                                           self.picker_tot_trays[self.pickers[i].picker_id],
                                                                                           self.picker_n_rows[self.pickers[i].picker_id],),
                                                               fontdict=self.font))
            else:
                self.picker_position_texts.append(self.ax.text(self.picker_x[self.pickers[i].picker_id] -0.75,
                                                               self.picker_y[self.pickers[i].picker_id] + 0.3,
                                                               "P_%s" %(self.pickers[i].picker_id[-2:]), fontdict=self.font))

    def set_picker_pose_subs(self, ):
        for i in range(self.n_pickers):
            self.picker_pose_subs.append(rospy.Subscriber("/%s/pose"%(self.pickers[i].picker_id),
                                                          geometry_msgs.msg.Pose,
                                                          self.update_picker_positions,
                                                          callback_args=self.pickers[i].picker_id))

    def set_picker_status_subs(self, ):
        for i in range(self.n_pickers):
            self.picker_pose_subs.append(rospy.Subscriber("/%s/status"%(self.pickers[i].picker_id),
                                                          rasberry_des.msg.Picker_Status,
                                                          self.update_picker_status,
                                                          callback_args=self.pickers[i].picker_id))

    def update_picker_positions(self, msg, picker_id):
        self.picker_x[picker_id] = msg.position.x
        self.picker_y[picker_id] = msg.position.y

    def update_picker_status(self, msg, picker_id):
        self.picker_picking_progress[picker_id] = msg.picking_progress
        self.picker_n_trays[picker_id] = msg.n_trays
        self.picker_tot_trays[picker_id] = msg.tot_trays
        self.picker_n_rows[picker_id] = msg.n_rows
        pass

    def plot_update(self, *args):
        for i in range(self.n_pickers):
            self.picker_position_lines[i].set_data(self.picker_x[self.pickers[i].picker_id],
                                                   self.picker_y[self.pickers[i].picker_id])
            if self.detailed:
                self.picker_position_texts.append(self.ax.text(self.picker_x[self.pickers[i].picker_id] -0.75,
                                                               self.picker_y[self.pickers[i].picker_id] + 0.3,
                                                               "P_%s\n%0.2f\n%d\n%d\n%d" %(self.pickers[i].picker_id[-2:],
                                                                                           self.picker_picking_progress[self.pickers[i].picker_id],
                                                                                           self.picker_n_trays[self.pickers[i].picker_id],
                                                                                           self.picker_tot_trays[self.pickers[i].picker_id],
                                                                                           self.picker_n_rows[self.pickers[i].picker_id],),
                                                               fontdict=self.font))
            else:
                self.picker_position_texts[i].set_position((self.picker_x[self.pickers[i].picker_id] -0.75,
                                                            self.picker_y[self.pickers[i].picker_id] + 0.3))
        self.fig.canvas.draw()



