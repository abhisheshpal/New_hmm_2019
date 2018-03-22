#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 21/02/2018
# @info: Picker - a simple picker class
# ----------------------------------

import math
import rospy
import tf
import geometry_msgs.msg
import rasberry_des.msg


class Picker(object):
    """Picker class definition"""
    def __init__(self, picker_id, env, farm, tray_capacity, max_n_trays,
                 picking_rate, transportation_rate, loading_time, sim_rt_factor=1.0):
        """Create a Picker object

        Keyword arguments:
        picker_id -- name/id of the picker
        env -- simpy.Environment
        farm -- provides the allocation and monitor process as well as a graph.
        try_capacity -- capacity of the tray picker is carrying
        max_n_trays -- number of trays with the picker
        picking_rate -- rate at which the picker moves while picking
        transportation_rate -- rate at which the picker moves while transporting
        loading_time -- time the picker will spend at the local storage for unloading
        """
        self.picker_id = picker_id
        self.env = env
        self.farm = farm
        self.n_rows = 0
        self.n_trays = 0     # current number of trays with the picker
        self.tot_trays = 0   # total number of trays by the picker
        self.tray_capacity = tray_capacity
        self.picking_rate = picking_rate * sim_rt_factor
        self.transportation_rate = transportation_rate * sim_rt_factor
        self.max_n_trays = max_n_trays
        self.max_wait_for_allocation = 5    # max time to wait before unloading everything

        self.loading_time = loading_time / sim_rt_factor    # time spent at localStorage
        self.assigned_row = None            # only for picking mode

        self.mode = 0       # 0:free, 1:picking, 2:transporting, 3:finished_job
        self.curr_node = None
        self.local_storage_node = None
        self.picking_dir = None     # "forward" or "reverse"

        self.curr_row = None
        self.prev_row = None

        # [head_node, start_node, end_node, row_node_dist, last_node_dist]
        self.curr_row_info = []
        self.prev_row_info = []

        self.row_path = []

        self.picking_progress = 0.  # percentage of tray_capacity

        self.prev_pub_time = 0.
        self.pub_delay = 0.1

        self.pose_pub = rospy.Publisher('/%s/pose' %(self.picker_id),
                                        geometry_msgs.msg.Pose,
                                        queue_size=10)
        self.status_pub = rospy.Publisher('/%s/status' %(self.picker_id),
                                        rasberry_des.msg.Picker_Status,
                                        queue_size=10)

        self.pose = geometry_msgs.msg.Pose()
        self.status = rasberry_des.msg.Picker_Status()
        self.status.picker_id = self.picker_id
        self.status.picking_rate = picking_rate
        self.status.transportation_rate = transportation_rate
        self.status.tray_capacity = self.tray_capacity
        self.status.picking_progress = 0.
        self.status.n_trays = 0
        self.status.tot_trays = 0
        self.status.n_rows = 0
        self.status.curr_row = "None"

        self.action = self.env.process(self.picking_process())

    def picking_process(self, ):
        """ Picker's picking process
        """
        # 1. picker should report for duty first
        self.farm.picker_report(self.picker_id)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]

        while True:
            # 2. If the picker is assigned a row,
            #   a. continue picking
            #   b. yield go_to_node(next_node) process
            #   c. update and check tray_cap,
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
                    # pick through to the next node
                    yield self.env.process(self.go_to_node(next_node, self.picking_rate))

                    # update the picking progress
                    self.picking_progress += self.farm.graph.yield_at_node[self.curr_node]

                    # reverse at the end
                    if self.curr_node == self.row_path[-1]:
                        self.picking_dir = "reverse"
                        rospy.loginfo("%s changing to reverse along %s at %0.3f" %(self.picker_id,
                                                                           self.curr_row,
                                                                           self.env.now))

                    # if the tray capacity is reached, increment n_trays
                    if self.picking_progress >= self.tray_capacity:
                        self.n_trays += 1
                        self.picking_progress -= self.tray_capacity

                    # if max_n_trays is reached
                    if self.n_trays == self.max_n_trays:
                        # if full rows, and at first or last row, and at the end node,
                        #   send row finished
                        #   go to local storage and no return
                        if ((not self.farm.half_rows) and
                            ((self.curr_row == self.farm.row_ids[0]) or
                             (self.curr_row == self.farm.row_ids[-1])) and
                            (self.curr_node == self.row_path[-1])):
                            # row is finished
                            self.finished_row_routine()
                            # transport to the local storage and don't return
                            yield self.env.process(self.transport_to_local_storage(item="tray"))
                            # finished the allocated row and transported full berry trays
                            # now at local_storage_node
                            # no current allocation - change mode back to zero
                            self.mode = 0
                        else:
                            curr_node = "" + self.curr_node # back up of self.curr_node
                            yield self.env.process(self.transport_to_local_storage(item="tray"))
                            # now at local storage, return to the curr_node
                            yield self.env.process(self.go_to_node(curr_node, self.transportation_rate))
                            # resume picking
                            self.mode = 1

                elif self.picking_dir is "reverse":
                    # work with negative indices
                    curr_node_index = self.row_path.index(self.curr_node) - len(self.row_path)
                    next_node = self.row_path[curr_node_index - 1]

                    if ((not self.farm.half_rows) and
                            ((self.curr_row == self.farm.row_ids[0]) or
                             (self.curr_row == self.farm.row_ids[-1])) and
                            (self.curr_node == self.row_path[-1])):
                        # there are full rows at the start and end
                        # row is finished
                        # navigate to the start node of the row and send row finish (no picking)
                        next_node = self.row_path[0] # finished_row_routine will reset row_path
                        self.finished_row_routine()
                        yield self.env.process(self.go_to_node(next_node, self.transportation_rate))
                    else:
                        # half rows at start and end (berries on both sides)
                        # pick through to the next node
                        yield self.env.process(self.go_to_node(next_node, self.picking_rate))
                        # update the picking progress
                        self.picking_progress += self.farm.graph.yield_at_node[self.curr_node]
                        if self.curr_node == self.row_path[0]:
                            # row is finished
                            self.finished_row_routine()

                    # check picking progress
                    if self.picking_progress >= self.tray_capacity:
                        self.picking_progress -= self.tray_capacity
                        self.n_trays += 1

                        if self.n_trays == self.max_n_trays:
                            if self.curr_row is None:
                                # finished picking along curr_row
                                # transport to local storage and don't return
                                yield self.env.process(self.transport_to_local_storage(item="tray"))
                                # reset mode to no allocations
                                self.mode = 0
                            else:
                                # transport to local storage and return
                                curr_node = "" + self.curr_node
                                yield self.env.process(self.transport_to_local_storage(item="tray"))
                                # return
                                yield self.env.process(self.go_to_node(curr_node, self.transportation_rate))
                                # resume picking
                                self.mode = 1

            # 3. If in mode free, check if there is any new assignments
            #       If there is no new assignment and no rows left to be assigned, finish picking
            #       If there is a new assignment
            #           a. Move to the start node of the path
            #           b. Get the path from the current loc to start_node of new row
            #           c. Move at transportation_rate to start_node
            #           d. Continue/Start picking along the new row by changing mode to picking
            elif self.mode == 0:
                row_id = self.farm.curr_picker_allocations[self.picker_id]
                row_id = None if row_id == self.prev_row else row_id

                if (row_id is None) and (len(self.farm.unallocated_rows) == 0):
                    # The picker has an assigned row and all rows are allocated
                    # unload any berries left in hand and leave the picking process
                    if self.curr_node == self.local_storage_node:
                        # at local storage after unloading max_n_trays
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.unload(item="all"))
                    elif self.curr_node is not None:
                        # the picker is at some node already
                        if (self.n_trays > 0) or (self.picking_progress > 0.):
                            yield self.env.process(self.transport_to_local_storage(item="all"))
                    self.mode = 3
                    # finish the picking process
                    rospy.loginfo("%s finishing picking process. all rows are assigned" %(self.picker_id))
                    break

                elif row_id is not None: # if there is a row assigned to the picker
                    self.curr_row = row_id
                    self.curr_row_info = self.farm.graph.row_info[self.curr_row]
                    # TODO: Now the local_storage_node of the first assigned row is assumed to be
                    # the starting position of the picker. Is an origin_node required?
                    if self.curr_node is None:
                        self.curr_node = self.farm.graph.local_storage_nodes[self.curr_row]
                        self.local_storage_node = self.farm.graph.local_storage_nodes[self.curr_row]

                    rospy.loginfo("%s is moving to the start of %s at %0.3f" %(self.picker_id,
                                                                       self.curr_row,
                                                                       self.env.now))
                    self.mode = 2
                    # go to the start_node of the row
                    yield self.env.process(self.go_to_node(self.curr_row_info[1], self.transportation_rate))

                    # picker moved to the start_node of the row (yield above)
                    # get the path from start to end of the row
                    self.row_path, _, _ = self.farm.graph.get_path_details(self.curr_node, self.curr_row_info[2])

                    rospy.loginfo("%s started forward picking on %s at %0.3f" %(self.picker_id, row_id, self.env.now))
                    # change current mode to picking
                    self.mode = 1 # picking mode

                    self.picking_dir = "forward"

            # publish pose
            if (self.curr_node is not None) and (self.env.now - self.prev_pub_time >= self.pub_delay):
                curr_node_obj = self.farm.graph.get_node(self.curr_node)
                position[0] = curr_node_obj.pose.position.x
                position[1] = curr_node_obj.pose.position.y
                self.publish_pose(position, orientation)

            yield self.env.timeout(0.001)

    def unload(self, item="tray"):
        """Picker's unloading process at the local storage node

        Keyword arguments:

        item -- unload "tray" or "all"; "tray" is normal, "all" only when no more rows are free
        """
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        curr_node_obj = self.farm.graph.get_node(self.curr_node)
        position[0] = curr_node_obj.pose.position.x
        position[1] = curr_node_obj.pose.position.y

        self.publish_pose(position, orientation)
        start_time = self.env.now
        delta_time = self.env.now - start_time

        rospy.loginfo("%s requesting for accessing the local storage %s at %0.1f" %(self.picker_id, self.local_storage_node, self.env.now))
        with self.farm.graph.local_storages[self.local_storage_node].request() as req:
            # wait for permission to acces local storage, no pose publishing in between
            yield req
            rospy.loginfo("%s was granted access to local storage %s at %0.1f" %(self.picker_id, self.local_storage_node, self.env.now))

            while delta_time <= self.loading_time:
                now_time = self.env.now
                if now_time - self.prev_pub_time >= self.pub_delay:
                    self.publish_pose(position, orientation)
                yield self.env.timeout(0.05)
                delta_time = now_time - start_time

        # update tot_trays
        if item == "tray":
            self.tot_trays += self.max_n_trays
            self.n_trays -= self.max_n_trays
        elif item == "all":
            self.tot_trays += self.n_trays + self.picking_progress / self.tray_capacity
            self.n_trays = 0
            self.picking_progress = 0

        rospy.loginfo("%s finished unloading at %0.1f" %(self.picker_id, self.env.now))
        rospy.loginfo("%s : tot_trays: %02d, n_trays: %02d, pick_progress: %0.3f" %(self.picker_id,
                                                                            self.tot_trays,
                                                                            self.n_trays,
                                                                            self.picking_progress))

        yield self.env.timeout(0.001)

    def publish_pose(self, position, orientation):
        """This method publishes the current position of the picker. Called only at nodes"""
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]
        quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1],
                                                              orientation[2])
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.pose)

        # update variables in status and publish
        self.status.picking_progress = self.picking_progress / self.tray_capacity
        self.status.n_trays = self.n_trays
        self.status.tot_trays = self.tot_trays
        self.status.n_rows = self.n_rows
        self.status.curr_row = "%s" %(self.curr_row)
        self.status_pub.publish(self.status)

        self.prev_pub_time = self.env.now

    def go_to_node(self, goal_node, nav_speed):
        """Simpy process to Mimic moving to the goal_node by publishing new position based on speed

        Keyword arguments:
        goal_node -- node to reach from current node
        """
        route_nodes, route_edges, route_distance = self.farm.graph.get_path_details(self.curr_node,
                                                                                    goal_node)
        position = [0., 0., 0.]
        orientation = [0., 0., 0.]
        for i in range(len(route_nodes) - 1):
            # move through each edge
            curr_node_obj = self.farm.graph.get_node(route_nodes[i])
            next_node_obj = self.farm.graph.get_node(route_nodes[i + 1])
            theta = math.atan2((next_node_obj.pose.position.y - curr_node_obj.pose.position.y),
                               (next_node_obj.pose.position.x - curr_node_obj.pose.position.x))

            position[0] = curr_node_obj.pose.position.x
            position[1] = curr_node_obj.pose.position.y

            edge_distance = route_distance[i]
            travel_time = edge_distance / nav_speed

            self.publish_pose(position, orientation)
            start_time = self.env.now
            delta_time = self.env.now - start_time

            while delta_time <= travel_time:
                delta = nav_speed * delta_time
                position[0] = curr_node_obj.pose.position.x + delta * math.cos(theta)
                position[1] = curr_node_obj.pose.position.y + delta * math.sin(theta)
                now_time = self.env.now
                if now_time - self.prev_pub_time >= self.pub_delay:
                    self.publish_pose(position, orientation)
                delta_time = now_time - start_time
                yield self.env.timeout(0.05)
            self.curr_node = route_nodes[i + 1]

        yield self.env.timeout(0.001)

    def transport_to_local_storage(self, item="tray"):
        """Transport item to local storage by yielding to go_to_node and unload processes

        Keyword arguments:

        item -- unload "tray" or "all"; "tray" is normal, "all" only when no more rows are free
        """
        # transport to the local storage and don't return
        self.mode = 2
        rospy.loginfo("%s reached %d trays. going to local storage at %0.3f" %(self.picker_id,
                                                                       self.max_n_trays,
                                                                       self.env.now))
        yield self.env.process(self.go_to_node(self.local_storage_node, self.transportation_rate))

        # reached local storage, now unload
        yield self.env.process(self.unload(item))

    def finished_row_routine(self, ):
        """Common things to do when picking along the allocated row is finished
        """
        # trigger finished_rows event for the curr_row
        self.farm.finished_rows[self.curr_row].succeed(value=self.env.now)
        # some attributes are reset
        self.picking_dir = None
        self.prev_row = "" + self.curr_row
        self.prev_row_info = [] + self.curr_row_info
        self.curr_row = None
        self.row_path = []
        self.n_rows += 1
        self.mode = 0