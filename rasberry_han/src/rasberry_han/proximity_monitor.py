import numpy as np
import rospy
import rospkg
import tf
import json
from bayes_people_tracker.msg import PeopleTracker
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, Pose
from nav_msgs.msg import Path
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from rasberry_han.cfg import HumanProximityMonitorConfig
from rasberry_han.safe_actions import TopicPublisherPolicy, DynamicReconfigurePolicy, DynamicReconfigureServicePolicy

class ProximityMonitor:
    def __init__(self):
        rospy.init_node("people_navigation_monitor")
        self._danger_ring_radius = 1.0
        self._warning_ring_radius = 2.0
        self._restric_fov = True
        self._current_policy = None
        self._stop_policy = TopicPublisherPolicy()
        self._warn_policy = DynamicReconfigureServicePolicy()
        self._not_restricted_edges = self.get_restricted_edges()
        #TODO enum
        self._current_protection_ring = 2
        self._last_time_received = rospy.get_time()
        self._timeout = rospy.get_param("~timeout_limit", 5)#timeout before it resumes motion
        #range plus min yaw_range
        self._restricted_yaw_range = rospy.get_param('~restricted_yaw_range', -0.3)
        self._offset = 0
        self._detected_person_publisher = rospy.Publisher("closest_person_orientation", PoseStamped, queue_size=5)
        self._range_visualizer_publisher = rospy.Publisher("person_range", PoseArray, queue_size=5)

        self._tf_listener = tf.TransformListener()
        self._dynamic_reconfigure_client = Server(HumanProximityMonitorConfig, self.dynamic_reconfigure_cb)

        rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.path_cb)
        rospy.Subscriber("current_edge", String, self.current_edge_cb)
        rospy.Subscriber("people_detector", PeopleTracker, self.people_tracker_cb, queue_size=1)

    def path_cb(self, path):
        self._tf_listener.waitForTransform("base_link", path.header.frame_id, rospy.Time(0),rospy.Duration(2.0))
        #person position on base_link frame
        #HACK taking last goal direction of x on base_link frame
        p=self._tf_listener.transformPose("base_link",path.poses[-1])

        if p.pose.position.x < 0:
            self._offset = np.pi
        else:
            self._offset = 0

    def dynamic_reconfigure_cb(self, config, level):
        rospy.logwarn("HumanProximityMonitor being configured")
        self._timeout = config.timeout
        self._danger_ring_radius = config.danger_region_radius
        self._warning_ring_radius = config.warning_region_radius
        self._restricted_yaw_range = config.angle_fov
        return config

    def get_restricted_edges(self):
        cfg_package = rospy.get_param("~config_package", "rasberry_optimise")
        config_directory = rospy.get_param("~config_directory", "/resources/")
        config_filename = rospy.get_param("~config_filename", "group_irn_omni.json")

        file = rospkg.RosPack().get_path(cfg_package) + config_directory + config_filename
        rospy.logwarn(file)
        with open(file) as f:
            edges_ids = json.load(f)
        return edges_ids

    def current_edge_cb(self, msg):
        if msg.data == "none":
            return
        if not msg.data.split('--')[0] in self._not_restricted_edges:
            rospy.logwarn("FULL FOV")
            self._restric_fov = False
        else:
            rospy.logwarn("Restricting FOV")
            self._restric_fov = True

    def skip_person(self, x,y):
        yaw = np.arctan(y/x)

        if self._restric_fov: #if current_edge on restrited Edges
            #Check range... if person not in range skip.... (Person detected aside the robot)
            if np.fabs(yaw) > self._offset + self._restricted_yaw_range:
                rospy.logdebug("Skip person")
                return True

        #publish orientation to closest person
        fb_msg = PoseStamped()
        fb_msg.header.stamp = rospy.Time.now()
        fb_msg.header.frame_id = "base_link"
        fb_msg.pose.orientation = self.quat_from_yaw(yaw)
        self._detected_person_publisher.publish(fb_msg)
        return False

    def quat_from_yaw(self, yaw):
        quaternion = Quaternion()
        tmp_quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        quaternion.x = tmp_quat[0]
        quaternion.y = tmp_quat[1]
        quaternion.z = tmp_quat[2]
        quaternion.w = tmp_quat[3]
        return quaternion

    def people_tracker_cb(self, msg):
        self._last_time_received = rospy.get_time()

        self.publish_range()
        warning_zone = False

        rospy.logdebug(self._current_protection_ring)
        for person in msg.poses: #iterate for each detected person
            #convert pose to "base_link"
            self._tf_listener.waitForTransform("base_link", msg.header.frame_id, rospy.Time(0),rospy.Duration(2.0))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.header.stamp = rospy.Time(0)
            pose_stamped.pose = person
            #person position on base_link frame
            p=self._tf_listener.transformPose("base_link",pose_stamped)

            #get Id of the region the person is.
            person_ring = self.get_ring(p.pose.position.x, p.pose.position.y)
            #If too close lock multiplexer
            if person_ring == 0:
                if not self.skip_person(p.pose.position.x, p.pose.position.y): #filter according edge
                    if self._current_protection_ring != 0 and self._current_policy is not None:
                        self._current_policy.stop()

                    self._current_policy = self._stop_policy
                    self._current_policy.execute()
                    self._current_protection_ring = 0
                    return
            #warning zone flag
            if person_ring == 1:
                if not self.skip_person(p.pose.position.x, p.pose.position.y): #filter according edge
                    warning_zone = True

        #if warning zone detected and _current_policy is not initialized
        if warning_zone:
            if self._current_policy is not None and self._current_protection_ring == 0:
                self._current_policy.stop()

            if self._current_protection_ring != 1:
                self._current_policy = self._warn_policy
                self._current_policy.execute()
                self._current_protection_ring = 1
            return

        #if person is out of warning zone and policy is initialized
        if not warning_zone:
            if self._current_policy is not None:
                self._current_policy.stop()
            self._current_policy = None
            self._current_protection_ring = 2

    def publish_range(self):
        #After all is process sometime to publish range
        #TODO find a better way to visualize the range Range Message not an option
        fov_msg = PoseArray()
        #TODO this frame might be a param
        fov_msg.header.frame_id = "base_link"
        fov_msg.header.stamp = rospy.Time.now()
        #Min RANGE
        p = Pose()
        p.orientation = self.quat_from_yaw(-(self._restricted_yaw_range) + self._offset)
        fov_msg.poses.append(p)
        p = Pose()
        p.orientation = self.quat_from_yaw(self._restricted_yaw_range + self._offset)
        fov_msg.poses.append(p)
        self._range_visualizer_publisher.publish(fov_msg)

    def get_ring(self, x, y):
        distance = np.hypot(x,y)

	if distance <= self._danger_ring_radius:
	    return 0

	if distance <= self._warning_ring_radius:
            return 1

	return 2


    def reset(self):
        if self._current_policy is not None:
            rospy.logwarn("no people detection received.... ENABLING NAVIGATION")
            self._current_policy.stop()
            self._current_policy = None
            self._current_protection_ring = 2

    def check_rate(self):
        if rospy.get_time() - self._last_time_received > self._timeout:
            self.reset()
