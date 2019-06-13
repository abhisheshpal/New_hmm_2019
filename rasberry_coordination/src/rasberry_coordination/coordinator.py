#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
# @date:
# ----------------------------------

import operator
import Queue
import copy

import rospy

import std_msgs.msg
import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.msg
import topological_navigation.route_search
import topological_navigation.tmap_utils

import rasberry_coordination.robot
import rasberry_coordination.srv


class Coordinator:
    """
    """
    def __init__(self, local_storage, charging_node, base_stations, robot_ids, max_task_priorities):
        """
        """
        self.ns = "/rasberry_coordination/"

        self.robot_ids = robot_ids
        # TODO:
        # Assuming only one local storage
        # Assuming each robot has a base station node
        # Assuming one charging node
        self.storage = local_storage
        self.charging_node = charging_node
        self.base_stations = base_stations
        self.max_task_priorities = max_task_priorities

        # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
        # 3 - waiting for unloading, 4 - transporting to storage, 5- charging
        # 6 - return to base from storage
        self.robot_states_str = {0:"Idle", 1:"Going to picker", 2:"Waiting for loading",
                             3:"Waiting for unloading", 4:"Going to storage",
                             5:"Charging", 6:"Going to base", 9:"Stuck"}
        self.robot_states = {robot_id:0 for robot_id in self.robot_ids}
        self.robots = {robot_id: rasberry_coordination.robot.Robot(robot_id) for robot_id in self.robot_ids}

        # collect_tray_stages = ["go_to_picker", "wait_loading", "go_to_storage", "wait_unloading", "got_to_base"]
        self.routes = {robot_id:[] for robot_id in self.robot_ids}
        self.route_edges = {robot_id:[] for robot_id in self.robot_ids}
        self.route_fragments = {robot_id:[] for robot_id in self.robot_ids}
        self.edge_policy_routes = {} # {robot_id: }
        self.critical_points = []
        self.task_stages = {robot_id: None for robot_id in self.robot_ids} # keeps track of current stage of the robot
        self.start_time = {robot_id: rospy.get_rostime() for robot_id in self.robot_ids}

        self.idle_robots = self.robot_ids + [] # a copy of robot_ids
        self.active_robots = [] # all robots executing a task
        self.moving_robots = [] # all robots which are having an active topo_nav goal (not waiting before critical points for clearance)

        self.presence_agents = rospy.get_param("topological_map_manager/agents", [])
        self.current_nodes = {agent_name:"none" for agent_name in self.presence_agents}
        self.prev_current_node = {agent_name:"none" for agent_name in self.presence_agents}
        self.closest_nodes = {agent_name:"none" for agent_name in self.presence_agents}
        self.current_node_subs = {agent_name:rospy.Subscriber(agent_name.strip()+"/current_node",
                                                              std_msgs.msg.String,
                                                              self.current_node_cb,
                                                              callback_args=agent_name) for agent_name in self.presence_agents}
        self.closest_node_subs = {agent_name:rospy.Subscriber(agent_name.strip()+"/closest_node",
                                                              std_msgs.msg.String,
                                                              self.closest_node_cb,
                                                              callback_args=agent_name) for agent_name in self.presence_agents}

        self.max_task_priorities = max_task_priorities

        self.advertise_services()
        # don't queue more than 1000 tasks
        self.tasks = Queue.PriorityQueue(maxsize=1000)
        self.last_id = 0
        # track the tasks to be cancelled
        self.to_be_cancelled = [] # task_ids to be cancelled

        self.all_task_ids = []
        self.processing_tasks = {}
        self.completed_tasks = {}
        self.cancelled_tasks = {}
        self.failed_tasks = {}

        self.task_robot = {}
        self.robot_task_id = {robot_id: None for robot_id in self.robot_ids}
        self.task_state_msg = rasberry_coordination.msg.TaskUpdates()
        self.tray_loaded_srv = rospy.Service(self.ns + "tray_loaded", rasberry_coordination.srv.TrayLoaded, self._tray_loaded_cb)
        self.tray_loaded = {robot_id:False for robot_id in self.robot_ids}

        # TODO: these durations should come from a config
        self.max_load_duration = 60.0
        self.max_unload_duration = 20.0

        # return a list of (key, value) tuples sorted based on values
        self.closest_robot = lambda x: sorted(x.items(), key=operator.itemgetter(1))

        self.tmap = None
        self.rec_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self.map_cb)
        rospy.loginfo("Waiting for Topological map ...")
        while not self.rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        rospy.loginfo("Received for Topological map ...")

        self.avail_tmap = None
        self.avail_rec_map = False
        rospy.Subscriber("available_topological_map", strands_navigation_msgs.msg.TopologicalMap, self.avail_map_cb)
        rospy.loginfo("Waiting for Available topological map ...")
        while not self.avail_rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        rospy.loginfo("Received for Available topological map ...")

        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

        self.picker_task_updates_pub = rospy.Publisher("/picker_state_monitor/task_updates", rasberry_coordination.msg.TaskUpdates, queue_size=5)

        rospy.loginfo("coordinator initialised")

    def map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_map = True

    def avail_map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.avail_topo_map = msg
        self.avail_rec_map = True

    def current_node_cb(self, msg, agent_name):
        """
        """
        if self.current_nodes[agent_name] != "none":
            self.prev_current_node[agent_name] = self.current_nodes[agent_name]
        self.current_nodes[agent_name] = msg.data

    def closest_node_cb(self, msg, agent_name):
        """
        """
        self.closest_nodes[agent_name] = msg.data

    def _tray_loaded_cb(self, req):
        """callback for tray_loaded service
        """
        self.tray_unloaded[req.robot_id] = False
        self.tray_loaded[req.robot_id] = True
        return (True, "")

    def add_task_ros_srv(self, req):
        """Adds a task into the task execution framework.
        """
        self.last_id += 1
        task_id = self.last_id
        req.task.task_id = task_id
        rospy.loginfo('received task: %s to %s', req.task.task_id, req.task.start_node_id)
        self.tasks.put(
            (task_id, req.task)
        )
        self.all_task_ids.append(task_id)
        return task_id

    add_task_ros_srv.type = strands_executive_msgs.srv.AddTask

    def cancel_task_ros_srv(self, req):
        """cancels a task from execution
        """
        cancelled = False
        # Two scenarios:
        # 1. task is already being processed
        #    this also implies the robot has not reached the picker, as
        #    the CAR interface to cancel task won't be available after that.
        #    the topo_nav goal to the robot has to be cancelled
        #    the robot will have to be sent to the base after the cancellation
        # 2. task is still queued
        #    pop the task from the queue

        if req.task_id in self.all_task_ids:
            if ((req.task_id in self.completed_tasks) or
                  (req.task_id in self.cancelled_tasks) or
                  (req.task_id in self.to_be_cancelled)):
                # cannot be here
                raise Exception("cancel_task_ros_srv cannot be in this condition")
                pass
            elif req.task_id in self.processing_tasks:
                # task is being processed. remove it
                self.processing_tasks.pop(req.task_id)
                # cancel topo_nav and return status
                if req.task_id in self.task_robot:
                    robot_id = self.task_robot[req.task_id]
                    self.robots[robot_id].exec_policy.cancel_goal()
                    # TODO: sending the robot to base also needs planning
#                    self.task_stages[robot_id] = "go_to_base"
                rospy.loginfo("cancelling task-%d", req.task_id)
                cancelled = True
            else:
                # not yet processed, but will not be taken for processing
                self.to_be_cancelled.append(req.task_id)
                rospy.loginfo("cancelling task-%d", req.task_id)
                cancelled = True
        else:
            # invalid task_id
            rospy.logerr("cancel_task is invoked with invalid task_id")

        return cancelled

    cancel_task_ros_srv.type = strands_executive_msgs.srv.CancelTask

    def advertise_services(self):
        """Adverstise ROS services.
        Only call at the end of constructor to avoid calls during construction.
        """
        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(
                    self.ns+attr[:-8],
                    service.type,
                    service
                )
                rospy.loginfo('advertised %s', attr[:-8])

    def find_closest_robot(self, task):
        """find the robot closest to the task location (picker_node)
        # assign based on priority (0 - virtual pickers only, >=1 real pickers)

        Keyword arguments:

        task - strands_executive_msgs.Task
        """
        goal_node = task.start_node_id
        robot_dists = {}
        for robot_id in self.idle_robots:
            # ignore if the task priority is less than the min task priority for the robot
            if task.priority > self.max_task_priorities[robot_id]:
                continue

            start_node = self.closest_nodes[robot_id]
            if start_node =="none" or goal_node == "none" or start_node is None or goal_node is None:
                route_dists = [float("inf")]
            elif start_node != goal_node:
                route_nodes, route_edges, route_dists = self.get_path_details(start_node, goal_node)
            else:
                route_dists = [0]

            if abs(sum(route_dists)) != float("inf"):
                robot_dists[robot_id] = sum(route_dists)

        if len(robot_dists) == 0 or min(robot_dists.values()) == float("inf"):
            return None
        return self.closest_robot(robot_dists)[0][0]

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:
        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_search.search_route(start_node, goal_node)
        if route is None:
            rospy.loginfo("no route between %s and %s", start_node, goal_node)
            return ([], [], [float("inf")])
        route_nodes = route.source
        route_edges = route.edge_id

        edge_to_goal = self.get_edges_between_nodes(route_nodes[-1], goal_node)
        if len(edge_to_goal) != 0:
            route_edges.append(edge_to_goal[0])
            route_nodes.append(goal_node)
        else:
            return ([], [], [float("inf")])

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.get_distance_between_adjacent_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return topological_navigation.tmap_utils.get_node(self.topo_map, node)

    def get_distance_between_adjacent_nodes(self, from_node, to_node):
        """get_distance_between_adjacent_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node(from_node_obj, to_node_obj)

    def get_edges_between_nodes(self, from_node, to_node):
        """get_edges_between_nodes: Given names of two nodes, return the direct edges
        between their node objects. A wrapper for the get_edges_between function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        edge_ids = []
        edges = topological_navigation.tmap_utils.get_edges_between(self.topo_map, from_node, to_node)
        for edge in edges:
            edge_ids.append(edge.edge_id)
        return edge_ids

    def assign_tasks(self, ):
        """
        """
        trigger_replan = False
        # get all tasks from queue
        tasks = []
        task_priorities = {}
        while not rospy.is_shutdown():
            try:
                task_id, task = self.tasks.get(True, 1)
                if task_id in self.to_be_cancelled:
                    self.cancelled_tasks[task_id] = task
                    rospy.loginfo("ignoring cancelled task-%d", task_id)
                tasks.append((task_id, task))
            except Queue.Empty:
                break

        # high priority tasks first served
        # among equal priority, first came first served
        for (task_id, task) in tasks:
            if task.priority not in task_priorities:
                task_priorities[task.priority] = {task_id: task}
            else:
                task_priorities[task.priority][task_id] = task

        priorities = sorted(task_priorities, key=operator.itemgetter(0))
        for priority in priorities:
            # try to get a robot for each task
            task_ids = sorted(task_priorities[priority], key=operator.itemgetter(0))
            for task_id in task_ids:
                task = task_priorities[priority][task_id]
                # among equal priority, first came first served
                robot_id = self.find_closest_robot(task)
                if robot_id is None:
                    continue
                rospy.loginfo("selected robot-%s to task %d", robot_id, task_id)

                # trigger replan for any new assignment
                trigger_replan = True

                tasks.remove((task_id, task))
                self.active_robots.append(robot_id)
                self.idle_robots.remove(robot_id)
                self.task_stages[robot_id] = "go_to_picker"

                self.processing_tasks[task_id] = task
                self.task_robot[task_id] = robot_id
                self.robot_task_id[robot_id] = task_id

                self.task_state_msg.task_id = task_id
                self.task_state_msg.robot_id = self.task_robot[task_id]
                self.task_state_msg.state = "ACCEPT"
                self.picker_task_updates_pub.publish(self.task_state_msg)

        # putting unassigned tasks back in the queue
        for (task_id, task) in tasks:
            self.tasks.put((task_id, task))

        return trigger_replan

    def finish_route_fragment(self, robot_id):
        """
        """
        self.moving_robots.remove(robot_id)
        self.routes.pop(robot_id)
        self.route_fragments.pop(robot_id)
        self.robots[robot_id].execpolicy_result = None

    def finish_task_stage(self, robot_id, curr_stage=None):
        """
        """
        assert curr_stage in ["go_to_picker", "go_to_storage", "go_to_base"]
        next_stage = {"go_to_picker":"wait_loading", "go_to_storage":"wait_unloading", "go_to_base":None}
        self.task_stages[robot_id] = next_stage[curr_stage]
        self.finish_route_fragment(robot_id)
        self.start_time[robot_id] = rospy.get_rostime()

    def finish_task(self, robot_id):
        """
        """
        self.finish_task_stage(robot_id, "go_to_base")
        # move task from processing to completed
        self.task_stages[robot_id] = None
        task_id = self.robot_task_id[robot_id]
        self.robot_task_id[robot_id] = None
        self.completed_tasks[task_id] = self.processing_tasks.pop(task_id)
        # move robot from active to idle
        self.active_robots.remove(robot_id)
        self.idle_robots.append(robot_id)

    def set_task_failed(self, task_id):
        """set task state as failed
        """
        self.task_robot.pop(task_id)
        task = self.processing_tasks.pop(task_id)
        self.failed_tasks[task_id] = task

    def publish_task_state(self, task_id, robot_id, state):
        """
        """
        self.task_state_msg.task_id = task_id
        self.task_state_msg.robot_id = robot_id
        self.task_state_msg.state = "ARRIVED"
        self.picker_task_updates_pub.publish(self.task_state_msg)
        rospy.sleep(0.01)

    def readd_task(self, robot_id):
        """
        """
        # this task should be readded to the queue if not cancelled by the picker !!!
        task_id = self.robot_task_id[robot_id]
        if task_id in self.processing_tasks:
            rospy.loginfo("Assigned robot failed to reach picker, adding the task back into the queue")
            self.robot_task_id.pop(robot_id) # remove from assigned task
            self.task_robot.pop(task_id) # remove from the assigned robot
            task = self.processing_tasks.pop(task_id) # remove from processing tasks

            self.publish_task_state(task_id, robot_id, "CALLED")

            self.tasks.put(
                (task_id, task)
            )

        rospy.sleep(0.01)

    def handle_tasks(self):
        """

        """
        # trigger replan if there is a new task assignment, a robot waiting completed
        # or task update from a robot
        trigger_replan = False

        for robot_id in self.active_robots:
            task_id = self.robot_task_id[robot_id]

            if robot_id in self.moving_robots:
                # topo nav stage
                # if any robot has finished its current goal, remove the finished goal from the robot's route
                if self.robots[robot_id].execpolicy_result is None:
                    # task/fragment not finished
                    continue

                # check for robots which are moving, not waiting before a critical point
                if self.robots[robot_id].execpolicy_result.success:
                    # trigger replan whenever a segment comppletion is reported
                    trigger_replan = True
                    # if the robot's route is finished, progress to the next stage of the collect tray process
                    # has it finished the stage?
                    goal_node = None
                    if self.task_stages[robot_id] == "go_to_picker":
                        # goal is picker node as in the task
                        goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
                    elif self.task_stages[robot_id] == "go_to_storage":
                        # goal is storage node
                        goal_node = self.storage
                    elif self.task_stages[robot_id] == "go_to_base":
                        # goal is robot's base node
                        goal_node = self.base_stations[robot_id]

                    if (len(self.route_fragments[robot_id]) == 1 and
                        self.current_nodes[robot_id] is not None and
                        self.current_nodes[robot_id] == goal_node):
                        # finished the stage
                        if self.task_stages[robot_id] == "go_to_picker":
                            # go_to_picker stage is finished
                            self.finish_task_stage(robot_id, "go_to_picker")
                            self.publish_task_state(task_id, robot_id, "ARRIVED")

                        elif self.task_stages[robot_id] == "go_to_storage":
                            # go_to_storage stage is finished
                            self.finish_task_stage(robot_id, "go_to_storage")
                            self.publish_task_state(task_id, robot_id, "STORAGE")

                        elif self.task_stages[robot_id] == "go_to_base":
                            # task is finished
                            self.finish_task(robot_id)

                    else:
                        # finished only a fragment. may have to wait for clearance
                        self.finish_route_fragment(robot_id)
                else:
                    # TODO: failed execution. robot will be stranded
                    print "%s failed to complete task %s at stage %s!!!" %(robot_id, task_id, self.task_stages[robot_id])
                    self.set_task_failed(task_id)
#                    if self.task_stages[robot_id] == "go_to_picker":
#                        pass
#                        # TODO: enable readd task
#                        # to enable readding, sending robot to base other than as part of a task stage needs to be implemented
##                        # readd task
##                        self.readd_task(robot_id)
##                        # send robot to base
#                    else:
#                        pass
            else:
                # wait_loading or wait_unloading
                if self.task_stages[robot_id] == "wait_loading":
                    # if conditions are satisfied, finish waiting
                    # 1. LOADED from CAR
                    # 2. service call from active_compliance
                    # 3. delay (?)
                    finish_waiting = False
                    if self.tray_loaded[robot_id]:
                        finish_waiting = True
                    elif False:
                        # active compliance
                        pass
                    elif rospy.get_rostime() - self.start_time[robot_id] > self.max_load_duration:
                        # delay
                        finish_waiting = True

                    if finish_waiting:
                        self.publish_task_state(task_id, robot_id, "LOADED")

                        self.task_stages[robot_id] = "go_to_storage"
                        self.finish_route_fragment(robot_id)
                        self.start_time[robot_id] = rospy.get_rostime()
                        trigger_replan = True

                elif self.task_stages[robot_id] == "wait_unloading":
                    # if conditions satisy, finish waiting
                    # 1. delay
                    if rospy.get_rostime() - self.start_time[robot_id] > self.max_unload_duration:
                        # delay
                        self.publish_task_state(task_id, robot_id, "DELIVERED")

                        self.task_stages[robot_id] = "go_to_base"
                        self.finish_route_fragment(robot_id)
                        self.start_time[robot_id] = rospy.get_rostime()
                        trigger_replan = True

                else:
                    # robot is waiting before a critical point
                    if not self.moving_robots:
                        # no other moving robots - replan
                        trigger_replan = True
                    else:
                        # should wait until one of the moving robots to finish its route fragment
                        pass

        return trigger_replan

    def critical_points(self, ):
        """
        """
        critical_points = {}
        for agent_id in self.persistent_agents:
#            if agent_id in self.robot_ids and self.task_stages[agent_id] in ["wait_loading", "wait_unloading"]:
#                continue
            r_outer = self.routes[agent_id]
            critical_points[str(r_outer)] = set([])
            for r_inner in self.routes[agent_id]:
                if r_outer is not r_inner:
                    #print(set(r_outer).intersection(set(r_inner)))
                    critical_points[str(r_outer)] = critical_points[str(r_outer)].union(set(r_outer).intersection(set(r_inner)))

        return critical_points

    def split_critical_paths(self, ):
        """
        """
        cp = self.critical_points()
        res_routes = {}
        for agent_id in self.persistent_agents:
#            if agent_id in self.robot_ids and self.task_stages[agent_id] in ["wait_loading", "wait_unloading"]:
#                continue

            r = self.routes[agent_id]
            rr = []
            partial_route = []
            for v in r:
                if v in cp[str(r)]: # vertice is critical point
                    if partial_route:
                        rr.append(partial_route)
                    partial_route = [v]
                else:
                    partial_route.append(v)

            if partial_route:
                rr.append(partial_route)
            res_routes[agent_id] = rr

        self.route_fragments = res_routes

        res_edges = {}
        # split the edges as per the route_fragments
        for robot_id in self.active_robots:
            if self.route_fragments[robot_id]:
                # remove goal node from last fragment
                # if start and goal nodes are different, there will be at least one node remaining and an edge
                self.route_fragments[robot_id][-1].pop(-1)
                # move the last node of all fragments to the start of next fragment
                for i in range(len(self.route_fragments[robot_id]) - 1):
                    self.route_fragments[robot_id][i+1].insert(0, self.route_fragments[robot_id][i][-1])
                    self.route_fragments[robot_id][i].pop(-1)
                    # split the edges
                    res_edges[robot_id] = self.route_edges[robot_id][:len(self.route_fragments[robot_id][i])]
                    self.route_edges[robot_id] = self.route_edges[robot_id][len(self.route_fragments[robot_id][i]):]
            else:
                print self.route_fragments[robot_id]

        self.route_edges = res_edges

    def set_execute_policy_route(self, ):
        """find connecting edges for each fragment in route_fragments and set
        the corresponding route object (with source and edge_id)
        """
        for robot_id in self.active_robots:
            goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
            if self.route_fragments[robot_id]:
                goal.route.source = self.route_fragments[robot_id][0]
                goal.route.edge_id = self.route_edges[robot_id][0]
            self.robots[robot_id].set_execpolicy_goal(goal)
            # will this be blocked here???
            # after sending goal to one robot, will the execution wait here because of send goal and wait ?
            print rospy.get_rostime()

    def replan(self, ):
        """
        """
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                avail_map = copy.deepcopy(self.avail_topo_map)
                avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_map)
                if self.task_stages[robot_id] in ["wait_loading", "wait_unloading"]:
                    # loading and unloading robots should finish those stages first
                    continue
                start_node = self.current_nodes[robot_id] if self.current_nodes[robot_id] != "none" else self.prev_current_nodes[robot_id]
                goal_node = None
                if self.task_stages[robot_id] == "go_to_picker":
                    # goal is picker node as in the task
                    avail_map = copy.deepcopy(self.avail_topo_map)
                    avail_route_search = topological_navigation.route_search.TopologicalRouteSearch(avail_map)
                    goal_node = self.processing_tasks[self.robot_task_id[robot_id]].start_node_id
                elif self.task_stages[robot_id] == "go_to_storage":
                    # goal is storage node
                    goal_node = self.storage
                elif self.task_stages[robot_id] == "go_to_base":
                    # goal is robot's base node
                    goal_node = self.base_stations[robot_id]

                route = avail_route_search.search_route(start_node, goal_node)
                route_nodes = []
                route_edges = []
                if route is None:
                    rospy.loginfo("no route between %s and %s", start_node, goal_node)
                else:
                    route_nodes = route.source
                    route_edges = route.edge_id
                    edge_to_goal = self.get_edges_between_nodes(route_nodes[-1], goal_node)
                    if len(edge_to_goal) != 0:
                        route_nodes.append(goal_node)

                self.routes[robot_id] = route_nodes
                self.route_edges[robot_id] = route_edges

            else:
                # put the current node of the idle robots as their route - to avoid other robots planning routes through those nodes
                self.routes[robot_id] = [self.current_nodes[robot_id] if self.current_nodes[robot_id] != "none" else self.prev_current_nodes[robot_id]]
                self.route_edges[robot_id] = []

        for agent_id in self.presence_agents:
            if agent_id not in self.robot_ids:
                # all robot_ids have routes are already defined
                self.routes[agent_id] = [self.current_nodes[agent_id] if self.current_nodes[agent_id] != "none" else self.prev_current_nodes[agent_id]]

        # find critical points and fragment routes to avoid critical point collistions
        self.split_critical_paths()

    def run(self):
        """the main loop of the coordinator
        """
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            trigger_replan_1 = False
            trigger_replan_2 = False

#            rospy.loginfo(self.idle_robots)
            if self.idle_robots and not self.tasks.empty():
                rospy.loginfo("unassigned tasks present. no. idle robots: %d", len(self.idle_robots))
                # try to assign all tasks
                trigger_replan_1 = self.assign_tasks()

            # check progress of active robots
            trigger_replan_2 = self.handle_tasks()
            # replan if needed
            if trigger_replan_1 or trigger_replan_2:
                # check for critical points replan
                self.replan()
                # assign first fragment of each robot
                self.set_execute_policy_route()

                # TODO: if the robot was moving and now there are no goals (route fragment):
                # send current node as the goal ?
                pass

    def on_shutdown(self, ):
        print "shutting down all actions"
        for robot_id in self.robot_ids:
            if robot_id in self.active_robots:
                self.robots[robot_id]._exec_policy.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('simple_task_coordinator', anonymous=True)
    c = Coordinator()
    c.run()
    rospy.spin()
