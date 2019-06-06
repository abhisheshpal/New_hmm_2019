#!/usr/bin/python
import rospy
from itertools import cycle
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from bayes_people_tracker.msg import PeopleTracker

class PersonPublisher:
    def __init__(self):
        rospy.init_node("fake_rviz_person_publisher")
        self._people_poses_ = PoseArray()
        self._people_tracker_ = PeopleTracker()
        self._people_poses_.header.frame_id = "map"
        self._people_tracker_.header.frame_id = "map"
        max_number_of_persons = 3

        for i in range(max_number_of_persons):
            p = Pose()
            p.orientation.w = 1.0
            self._people_poses_.poses.append(p)

        self._people_iterator = cycle(self._people_poses_.poses)

        self._people_pub_ = rospy.Publisher("people_positions", PoseArray, queue_size = 1)
        self._people_tracker_pub_ = rospy.Publisher("people_detector", PeopleTracker, queue_size = 1)
        rospy.Subscriber("clicked_point", PointStamped, self.point_cb)
        #rospy.spin()

    def point_cb(self, msg):
        self._people_poses_.header.stamp = rospy.Time.now()
        self._people_tracker_.header.stamp = rospy.Time.now()

        self._people_iterator.next().position = msg.point
        self._people_tracker_.poses = self._people_poses_.poses

    def publish_topics(self):
        self._people_pub_.publish(self._people_poses_)
        self._people_tracker_pub_.publish(self._people_tracker_)


if __name__ == '__main__':
    person_publisher = PersonPublisher()
    last_time_publish = rospy.get_time()

    while not rospy.is_shutdown():
        if rospy.get_time() - last_time_publish > 1:
            person_publisher.publish_topics()
            last_time_publish = rospy.get_time()
