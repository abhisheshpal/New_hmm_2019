#!/usr/bin/env python

# ----------------------------------
# @author: Vicky
# @email: engr.electron@gmail.com
# ----------------------------------


# This script reads the data from hedge_pos_a and publishes it as PeopleStamped msg and PoseArray. 
# The script will continously look for active marvelmind devices and if a device does not update its location in 
# <kickout_time> seconds, will be deleted from PeopleStamped msg and PoseArray.
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import rospy
import marvelmind_nav.msg
from people_msgs.msg import Person
from people_msgs.msg import People

peoplearray = People()
posearray = PoseArray()
marvelmind_people_list = []
kickout_time = 5

class myclass(object):
    def __init__(self,marvelmind_topic):
        self.marvelmind_topic = marvelmind_topic
        self.pub = rospy.Publisher('marvelmind_positions',People,queue_size=10)
	self.pub2 = rospy.Publisher('marvelmind_posearray',PoseArray,queue_size=10)
        rospy.Subscriber(marvelmind_topic,marvelmind_nav.msg.hedge_pos_a,self.get_locations)
        rospy.Timer(rospy.Duration(kickout_time), self.marvelmind_active, oneshot=False)  #gps_active will be called after every <kickout_time> seconds

    def get_locations(self,msg):
        now = rospy.get_rostime()
        peoplearray.header.frame_id='/map'
        peoplearray.header.stamp=now 
        name = str(msg.address)       #Name of the person
        posearray.header.frame_id='/map'
        posearray.header.stamp=now
        if name not in marvelmind_people_list:
            marvelmind_people_list.append(name)
            ### Populating People message #######
            self.person=Person()   
            self.person.name = name
            self.person.position.x = msg.x_m
            self.person.position.y = msg.y_m
            self.person.position.z = 1
            self.person.reliability = now.secs
            peoplearray.people.append(self.person)
	    ### Populating PoseArray ###########
            self.pose=Pose()   
            self.pose.position.x = msg.x_m
            self.pose.position.y = msg.y_m
            self.pose.position.z = 1
            posearray.poses.append(self.pose)
        elif name in marvelmind_people_list:
            idx=marvelmind_people_list.index(name)
            peoplearray.people[idx].position.x = msg.x_m
            peoplearray.people[idx].position.y = msg.y_m
            peoplearray.people[idx].position.z = 1
            peoplearray.people[idx].reliability = now.secs
            posearray.poses[idx].position.x = msg.x_m
            posearray.poses[idx].position.y = msg.y_m
            posearray.poses[idx].position.z = 1
        else:
            pass
        self.pub.publish(peoplearray)
        self.pub2.publish(posearray)

    def marvelmind_active(self,mydata):
        num_of_marvelminds = len(marvelmind_people_list)             #Number of active marvelmind users
        current_time=rospy.get_rostime()
        for i in range (num_of_marvelminds-1,-1,-1):
            if len(marvelmind_people_list)>0 and (current_time.secs-peoplearray.people[i].reliability)>kickout_time:
                peoplearray.people.pop(i)
                posearray.poses.pop(i)
                marvelmind_people_list.pop(i)
if __name__ == '__main__':
    rospy.init_node('marvelmind_localization',anonymous=True)
    sml=myclass("hedge_pos_a")
    rospy.spin()
