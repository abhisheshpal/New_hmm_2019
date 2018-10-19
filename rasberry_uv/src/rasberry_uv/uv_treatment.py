#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date: 01/10/2018
# ----------------------------------

import threading

import rospy
import actionlib
import std_msgs.msg
import topological_navigation.msg
import geometry_msgs.msg
import rasberry_people_perception.topological_localiser
import std_srvs.srv


class UVTreatment(object):
    def __init__(self, ns="/", use_sim=False):
        self.ns = ns
        self.use_sim = use_sim

        self.lock = threading.Lock()

        rospy.loginfo("Initialising TopologicalNavLoc object")
        self.topo_localiser = rasberry_people_perception.topological_localiser.TopologicalNavLoc()
        rospy.loginfo("TopologicalNavLoc object ready")
        rospy.loginfo("Initialising UVTreatment object")

        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "map"
        self._pose_sub = rospy.Subscriber(self.ns + "robot_pose", geometry_msgs.msg.Pose, self._update_pose_cb)

        self.current_node = "none"
        self._current_node_sub = rospy.Subscriber(self.ns + "current_node", std_msgs.msg.String, self._update_current_cb)

        self.closest_node = "none"
        self._closest_node_sub = rospy.Subscriber(self.ns + "closest_node", std_msgs.msg.String, self._update_closest_cb)

        # topological navigation action client
        self._topo_nav = actionlib.SimpleActionClient(self.ns + "topological_navigation", topological_navigation.msg.GotoNodeAction)

        if not self.use_sim:
            self.uv_trigger_req = std_srvs.srv.SetBoolRequest()
            rospy.loginfo("Waiting for /switch_uv service...")
            rospy.wait_for_service("/switch_uv")
            self.uv_trigger_client = rospy.ServiceProxy("/switch_uv", std_srvs.srv.SetBool)

        rospy.loginfo("UVTreatment object ready")

    def _update_pose_cb(self, msg):
        """callback function to update robot_pose topics.
        it alse tries to update the closest_node if lock can be acquired
        """
        self.pose.pose = msg
        locked = self.lock.acquire(False)
        if locked:
            currentstr, closeststr = self.topo_localiser.localise_pose(self.pose)
            if currentstr != "none":
                self.current_node = currentstr
            if closeststr != "none":
                self.closest_node = closeststr
            self.lock.release()

    def _update_current_cb(self, msg):
        """callback function to update current_node topics.
        this is a latched topics and with multiple roscores, the updates may
        not be at an ideal rate which can cause problems for scheduling.
        so robot_pose subscriber callback also updates current_node.
        """
        locked = self.lock.acquire(False)
        if locked:
            if msg.data != "none":
                self.current_node = msg.data
            self.lock.release()

    def _update_closest_cb(self, msg):
        """callback function to update closest_node topics.
        this is a latched topics and with multiple roscores, the updates may
        not be at an ideal rate which can cause problems for scheduling.
        so robot_pose subscriber callback also updates closest_node.
        """
        locked = self.lock.acquire(False)
        if locked:
            if msg.data != "none":
                self.closest_node = msg.data
            self.lock.release()

    def _set_topo_nav_goal(self, goal_node, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal and set feedback and done callbacks to topo_nav action client
        """
        goal = topological_navigation.msg.GotoNodeGoal()
        rospy.loginfo("robot-%s has goal %s" %(self.ns[1:], goal_node))
        goal.target = goal_node
        goal.no_orientation = False
        self._topo_nav.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        self._topo_nav.wait_for_result()

    def _do_treatment(self, goal_node):
        """wrapper for sending specific goal to topo_nav action client
        """
        rospy.loginfo("send robot-%s to do uv treatment" %(self.ns[1:]))
        # TODO: uv_rig on service call
        rospy.loginfo("Turning UV Rig ON")
        if not self.use_sim:
            self.uv_trigger_req.data = True
            uv_trigger_res = self.uv_trigger_client.call(self.uv_trigger_req.data)
#            if uv_trigger_res.success:
#                rospy.loginfo("Turned UV Rig ON")

        self._set_topo_nav_goal(goal_node=goal_node,
                                done_cb=self._done_treatment_cb,
                                feedback_cb=self._fb_cb)

    def _fb_cb(self, fb):
        """topo_nav feedback callback
        """
        rospy.loginfo(fb)

    def _done_cb(self, status, result):
        """topo_nav done callback
        """
        if result is None:
            rospy.loginfo("failed to finish topo_navigation")
        elif result.success:
            rospy.loginfo("topo_navigation completed")
        elif not rospy.is_shutdown():
            rospy.loginfo("failed to finish topo_navigation")

    def _done_treatment_cb(self, status, result):
        """treatment topo_nav done callback
        """
        # TODO: uv_rig off service call
        rospy.loginfo("Turning UV Rig OFF")
        if not self.use_sim:
            self.uv_trigger_req.data = False
            uv_trigger_res = self.uv_trigger_client.call(self.uv_trigger_req.data)
#            if uv_trigger_res.success:
#                rospy.loginfo("Turned UV Rig ON")

        if result is None:
            rospy.loginfo("failed to finish topo_navigation")
        elif result.success:
            rospy.loginfo("topo_navigation completed")
        elif not rospy.is_shutdown():
            rospy.loginfo("failed to finish topo_navigation")

    def run(self, ):
        # the information about the rows, starting and finishing nodes should
        # ideally come from a yaml file
        rows = [2, 3]#, 4, 5]
        row_start_nodes = {2:"WayPoint64", 3:"WayPoint27", 4:"WayPoint59", 5:"WayPoint45"}
        row_finish_nodes = {2:"WayPoint10", 3:"WayPoint60", 4:"WayPoint36", 5:"WayPoint58"}
        # TODO: something to explore. Add new nodes in the rows at the start and end for
        # uv on/off purposes. current/closest node can then be used decide when this should
        # be done.
#        uv_on_nodes = {2:"WayPoint64", 3:"WayPoint27", 4:"WayPoint59", 5:"WayPoint45"}
#        uv_off_nodes = {2:"WayPoint10", 3:"WayPoint60", 4:"WayPoint36", 5:"WayPoint58"}

        for row_id in rows:
            # go to start of row
            self._set_topo_nav_goal(goal_node=row_start_nodes[row_id], done_cb=self._done_cb, feedback_cb=self._fb_cb)
            result = self._topo_nav.get_result()

            info_msg = "failed to reach start of row %d. exiting!" %(row_id)
            if result is None:
                rospy.loginfo(info_msg)
                return False
            elif not result.success:
                rospy.loginfo(info_msg)
                return False

            rospy.sleep(0.2)

            self._do_treatment(row_finish_nodes[row_id])
            result = self._topo_nav.get_result()

            info_msg = "failed to reach finish node of row %d. exiting!" %(row_id)
            if result is None:
                rospy.loginfo(info_msg)
                return False
            elif not result.success:
                rospy.loginfo(info_msg)
                return False

            rospy.sleep(0.2)

        # go to base node
        self._set_topo_nav_goal(goal_node="WayPoint56", done_cb=self._done_cb, feedback_cb=self._fb_cb)
        result = self._topo_nav.get_result()

        info_msg = "failed to reach base_node (WayPoint56). exiting!"
        if result is None:
            rospy.loginfo(info_msg)
            return False
        elif not result.success:
            rospy.loginfo(info_msg)
            return False

        rospy.sleep(0.2)
        return True

    def on_shutdown(self, ):
        """on ros shutdown - cancel all topo_nav goals
        """
        self._topo_nav.cancel_all_goals()
