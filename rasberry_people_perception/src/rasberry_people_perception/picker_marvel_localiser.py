#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# @info: this uses only localisation information from marvelmind ultrasonic nodes
# ----------------------------------


import rasberry_people_perception.topological_localiser
import rospy
import geometry_msgs.msg
import marvelmind_nav.msg
import std_msgs.msg
import tf

class PickerMarvelLocaliser(object):
    """A class to get picker positions from hedge_pos_a topics from marvelmind targets and the following
    TODO: (1) to transform them to /map frame and publish /pose for each picker
    (2) to localise each picker in the topological map and publish closest and current node topics
    """
    def __init__(self, hedge_pos_a_topic="/hedge_pos_a", hedge_pose_frame_id="marvelmind", global_frame_id="map"):
        """initialise PickerMarvelLocaliser object

        Keyword arguments:

        hedge_pos_a_topic - marvelmind_nav pos with address, default="/hedge_pos_a"
        hedge_pose_frame_id - frame_id of marvelmind_nav pos, default="/marvelmind"
        global_frame_id - global frame id, default="/map"
        """
        self.hedge_pose_frame_id = hedge_pose_frame_id
        self.global_frame_id = global_frame_id
        self.n_pickers = 0
        self.picker_marvel_ids = []

        self.posestamped_pubs = {} # publishers for /pose topic for each picker
        self.posestamped_msgs = {}
#        self.pose_msgs = {}

        self.closest_node_pubs = {} # publishers for /closest_node topic for each picker
        self.closest_node_msgs = {}
        self.current_node_pubs = {} # publishers for /current_node topic for each picker
        self.current_node_msgs = {}

        self.topo_localiser = rasberry_people_perception.topological_localiser.TopologicalNavLoc()

#        self.tf_listnener = tf.TransformListener()
#        self.tf_listnener.waitForTransform(self.global_frame_id, self.hedge_pose_frame_id, rospy.get_rostime(), rospy.Duration(10.0))

        # /hedge_pos_a subscriber should be after the transforListener to avoid errors in subscriber callback
        self.marvel_sub = rospy.Subscriber(hedge_pos_a_topic, marvelmind_nav.msg.hedge_pos_a, self.hedge_pos_a_cb)

    def hedge_pos_a_cb(self, msg):
        """callback function for headge_pose_a topics
        """
        if msg.address not in self.picker_marvel_ids:
            self.n_pickers += 1
            self.picker_marvel_ids.append(msg.address)
            # set up pose publishers
            self.posestamped_pubs[msg.address] = rospy.Publisher("/picker_%02d/posestamped" %(msg.address), geometry_msgs.msg.PoseStamped, queue_size=5)
#            self.pose_msgs[msg.address] = geometry_msgs.msg.Pose()
            self.posestamped_msgs[msg.address] = geometry_msgs.msg.PoseStamped()
            self.posestamped_msgs[msg.address].header.frame_id = "/map"
            # set up topo map related pubs
            self.current_node_pubs[msg.address] = rospy.Publisher("/picker_%02d/current_node" %(msg.address), std_msgs.msg.String, queue_size=5)
            self.current_node_msgs[msg.address] = std_msgs.msg.String()
            self.closest_node_pubs[msg.address] = rospy.Publisher("/picker_%02d/closest_node" %(msg.address), std_msgs.msg.String, queue_size=5)
            self.closest_node_msgs[msg.address] = std_msgs.msg.String()

        # TODO: assuming there is a tf frame /marvelmind from which a transformation is broadcasted to /map frame
        # TODO: pose in frame /map to be used for finding the topological current and closest nodes

        self.posestamped_msgs[msg.address].pose.position.x = msg.x_m
        self.posestamped_msgs[msg.address].pose.position.y = msg.y_m
        self.posestamped_pubs[msg.address].publish(self.posestamped_msgs[msg.address])
        current_node, closest_node = self.topo_localiser.localise_pose(self.posestamped_msgs[msg.address])
        if current_node is not None:
            self.current_node_msgs[msg.address].data = current_node
            self.current_node_pubs[msg.address].publish(self.current_node_msgs[msg.address])
        if closest_node is not None:
            self.closest_node_msgs[msg.address].data = closest_node
            self.closest_node_pubs[msg.address].publish(self.closest_node_msgs[msg.address])

