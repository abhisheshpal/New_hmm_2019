#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# @original-author: jailander@github for strands-project
# ----------------------------------

import rospy
import strands_navigation_msgs.msg
import strands_navigation_msgs.srv
import topological_navigation.tmap_utils

# TODO: This could be replaced in future if this class is available as a module
# from strands-project/strands_navigation/topological_navigation

def point_in_poly(node, pose):
    """Checks whether a given pose is within the polygon boundary of a node
    """
    x = pose.position.x-node.pose.position.x
    y = pose.position.y-node.pose.position.y

    n = len(node.verts)
    inside = False

    p1x = node.verts[0].x
    p1y = node.verts[0].y
    for i in range(n+1):
        p2x = node.verts[i % n].x
        p2y = node.verts[i % n].y
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


class TopologicalNavLoc(object):
    """this is a trimmed version of the TopologicalNavLoc class in
    topological_navigation/scripts/localisation.py by jailander@github
    """
    def __init__(self, ns="/"):
        self.ns = ns
        self.tmap = None
        self.rec_map = False
        self.nogos = []
        self.point_in_poly = point_in_poly

        #This service returns the closest node from a given pose
        rospy.wait_for_service("topological_map_manager/get_tagged_nodes")
        self.get_tagged_nodes = rospy.ServiceProxy("topological_map_manager/get_tagged_nodes", strands_navigation_msgs.srv.GetTaggedNodes)

        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self.map_cb)
        rospy.loginfo("Waiting for Topological map ...")

        while not self.rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))

    def localise_pose(self, req):
        """This function gets the node and closest node for a pose
        """
        not_loc = True
        distances = []
        distances = self.get_distances_to_pose(req.pose)
#        distances = self.get_distances_to_pose(req)
        closeststr = None
        currentstr = None

        ind = 0
        while not_loc and ind < len(distances) and ind < 3:
            if self.point_in_poly(distances[ind]["node"], req.pose):
#            if self.point_in_poly(distances[ind]["node"], req):
                currentstr = str(distances[ind]["node"].name)
                closeststr = currentstr
                not_loc = False
            ind += 1

        ind = 0
        while not_loc and ind < len(distances):
            if distances[ind]["node"].name not in self.nogos:
                closeststr = str(distances[ind]["node"].name)
                not_loc = False
            ind += 1

        return currentstr, closeststr


    def get_distances_to_pose(self, pose):
        """This function returns the distance from each waypoint to a pose in an organised way
        """
        distances = []
        for i in self.tmap.nodes:
            d = topological_navigation.tmap_utils.get_distance_node_pose(i, pose)
            a = {}
            a["node"] = i
            a["dist"] = d
            distances.append(a)

        distances = sorted(distances, key=lambda k: k["dist"])
        return distances

    def get_no_go_nodes(self):
        """This function gets the list of No go nodes
        """
        try:
            resp1 = self.get_tagged_nodes("no_go")
            return resp1.nodes

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

    def map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.tmap = msg
        self.rec_map = True

        self.nogos = []
        self.nogos = self.get_no_go_nodes()

