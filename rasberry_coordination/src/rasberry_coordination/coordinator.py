#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
# @date:
# ----------------------------------

import operator
import Queue

import rospy

import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.srv
import topological_navigation.route_search
import topological_navigation.tmap_utils

import rasberry_coordination.robot
import rasberry_coordination.srv


class Coordinator:
    """
    """
    def __init__(self, local_storage, charging_node, base_stations, picker_ids, robot_ids, max_task_priorities, unified=False):
        """
        """
        self.ns = "/rasberry_coordination/"

        self.picker_ids = picker_ids
#        self.robot_ids = robot_ids

        if unified:
            # create only one robot
            self.robot_ids = robot_ids[:1]
            self.robots = {robot_id:rasberry_coordination.robot.Robot(robot_id, local_storage, charging_node, base_stations[robot_id], unified) for robot_id in self.robot_ids}
        else:
            self.robot_ids = robot_ids
            self.robots = {robot_id:rasberry_coordination.robot.Robot(robot_id, local_storage, charging_node, base_stations[robot_id], unified) for robot_id in self.robot_ids}

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

        self.task_robot = {}

        # return a list of (key, value) tuples sorted based on values
        self.closest_robot = lambda x: sorted(x.items(), key=operator.itemgetter(1))

        self.tmap = None
        self.rec_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self.map_cb)
        rospy.loginfo("Waiting for Topological map ...")

        while not self.rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))

        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

        # TODO: there are service proxies in coordinator for services in picker_state_monitor and vice versa
        # if wait_for_service, it would be blocking
        self.picker_task_updates_pub = rospy.Publisher("/picker_state_monitor/task_updates", rasberry_coordination.msg.TaskUpdates, queue_size=5)

        rospy.loginfo("coordinator initialised")

    def map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_map = True

    def get_robot_state_ros_srv(self, req):
        """get the state of a robot"""
        resp = rasberry_coordination.srv.RobotStateResponse()
        if req.robot_id in self.robot_ids:
            resp.state, resp.goal_node, resp.start_time = self.robots[req.robot_id].get_state()
        else:
            err_msg = "%s is not a among the robots configured" %(req.robot_id)
            rospy.logerr(err_msg)
        return resp

    get_robot_state_ros_srv.type = rasberry_coordination.srv.RobotState

    def get_robot_states_ros_srv(self, req):
        """get the state of a set of robots"""
        resp = rasberry_coordination.srv.RobotStatesResponse()
        for robot_id in req.robot_ids:
            if robot_id in self.robot_ids:
                state, goal_node, start_time = self.robots[robot_id].get_state()
                resp.states.append(state)
                resp.goal_nodes.append(goal_node)
                resp.start_times.append(start_time)
            else:
                resp.states.append("")
                resp.goal_nodes.append("")
                resp.start_times.append(rospy.Time())
                err_msg = "%s is not a among the robots configured" %(robot_id)
                rospy.logerr(err_msg)
        return resp

    get_robot_states_ros_srv.type = rasberry_coordination.srv.RobotStates

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
                    self.robots[robot_id].collect_tray.cancel_goal()
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

    def handle_task(self):
        """this is to be overwritten if used as a superclass
        """
        pass

    def get_idle_robots(self, ):
        """get the list of robots which are idle
        """
        idle_robots = []
        for robot_id in self.robot_ids:
            if self.robots[robot_id].is_idle() and self.robots[robot_id].closest_node != "none":
                # if the robot is idle and its closest_node is not 'none' (topo_nav ready for that robot)
                idle_robots.append(robot_id)
            else:
                pass
        return idle_robots

    def get_stuck_robots(self, ):
        """get the list of robots which are stuck after their task was cancelled
        """
        stuck_robots = []
        for robot_id in self.robot_ids:
            self.robots[robot_id].get_state()

        return stuck_robots

    def find_closest_robot(self, task, idle_robots):
        """find the robot closest to the task location (picker_node)

        Keyword arguments:

        task - strands_executive_msgs.Task
        idle_robots - list of robot_ids
        """
        goal_node = task.start_node_id
        robot_dists = {}
        for robot_id in idle_robots:
            # ignore if the task priority is less than the min task priority for the robot
            if task.priority > self.max_task_priorities[robot_id]:
                continue

            start_node = self.robots[robot_id].closest_node
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

    def run(self):
        """the main loop of the coordinator
        """
        # get idle robots
        idle_robots = self.get_idle_robots()

        while not rospy.is_shutdown():
            rospy.sleep(0.01)
#            rospy.loginfo(idle_robots)
            if len(idle_robots) != 0 and not self.tasks.empty():
                rospy.loginfo("unassigned tasks present. no. idle robots: %d", len(idle_robots))

                # try assigning a robot if there is an idle robot
                try:
                    (task_id, task) = self.tasks.get(True, 1)
                    if task_id in self.to_be_cancelled:
                        self.cancelled_tasks[task_id] = task
                        rospy.loginfo("ignoring cancelled task-%d", task_id)
                    else:
                        rospy.loginfo('process task %d', task_id)
                        # find the closest robot
                        robot_id = self.find_closest_robot(task, idle_robots)
                        if robot_id is None:
                            rospy.loginfo("No free robot for task %d. Putting task back in the queue", task_id)
                            self.tasks.put(
                                (task_id, task)
                            )
                        else:
                            rospy.loginfo("selected robot-%s to task %d", robot_id, task_id)

                            self.processing_tasks[task_id] = task
                            self.task_robot[task_id] = robot_id

                            # call collecttray action -- action selection must be from the task details
                            collect_tray_goal = rasberry_coordination.msg.CollectTrayGoal()

                            collect_tray_goal.task = task
                            # hard coding duration now
                            collect_tray_goal.min_load_duration.secs = 10.
                            collect_tray_goal.max_load_duration.secs = 20.
                            collect_tray_goal.min_unload_duration.secs = 10.
                            collect_tray_goal.max_unload_duration.secs = 20.
                            self.robots[robot_id].collect_tray.send_goal(collect_tray_goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)

                            task_state = rasberry_coordination.msg.TaskUpdates()
                            task_state.task_id = task_id
                            task_state.robot_id = self.task_robot[task_id]
                            task_state.state = "ACCEPT"
                            self.picker_task_updates_pub.publish(task_state)

                except Queue.Empty:
                    pass

            # update idle robots
            rospy.sleep(0.01)
            idle_robots = self.get_idle_robots()

    def done_cb(self, status, result):
        """done callback for collect_tray action
        """
        if result.success:
            self.task_robot.pop(result.task_id)
            task = self.processing_tasks.pop(result.task_id)
            self.completed_tasks[result.task_id] = task
        else:
            # already updated based on feedback
            pass

    def feedback_cb(self, fb):
        """feedback callback for collect_tray action
        """
        if fb.route == "reached picker":
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "ARRIVED"
            self.picker_task_updates_pub.publish(task_state)
            rospy.sleep(0.01)

        elif fb.route == "tray loaded":
            # in case this is set by robot after a delay
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "LOADED"
            self.picker_task_updates_pub.publish(task_state)
            rospy.sleep(0.01)

        elif fb.route == "reached storage":
            # TODO: tray unloading is not considered
            # no need to do anything here at the moment
            pass

        elif fb.route == "tray unloaded":
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "DELIVERED"
            self.picker_task_updates_pub.publish(task_state)
            rospy.sleep(0.01)

        elif ((fb.route == "failed to reach the picker") or
              (fb.route == "failed to load tray")):
            # this task should be readded to the queue if not cancelled by the picker !!!
            if fb.task_id in self.processing_tasks:
                rospy.loginfo("Assigned robot failed to reach picker, adding the task back into the queue")
                robot_id = self.task_robot.pop(fb.task_id) # remove from the assigned robot
                task = self.processing_tasks.pop(fb.task_id) # remove from processing tasks

                task_state = rasberry_coordination.msg.TaskUpdates()
                task_state.task_id = fb.task_id
                task_state.robot_id = robot_id
                task_state.state = "CALLED"
                self.picker_task_updates_pub.publish(task_state)

                self.tasks.put(
                    (fb.task_id, task)
                )

            rospy.sleep(0.01)

        elif ((fb.route == "failed to reach the storage") or
              (fb.route == "failed to unload tray") or
              (fb.route == "failed to reach the base station")):
            # trays are already collected, so there is no point in assigning another robot
            # failed
            self.task_robot.pop(fb.task_id)
            task = self.processing_tasks.pop(fb.task_id)
            self.cancelled_tasks[fb.task_id] = task

    def on_shutdown(self, ):
        print "shutting down all actions"
        for robot_id in self.robot_ids:
            print robot_id, self.robots[robot_id].is_idle()
            if not self.robots[robot_id].is_idle():
                self.robots[robot_id]._topo_nav.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('simple_task_coordinator', anonymous=True)
    c = Coordinator()
    c.run()
    rospy.spin()
