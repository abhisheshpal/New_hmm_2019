#!/usr/bin/env python


#----------------------------------
# @author: Vicky
# @email: engr.electron@gmail.com
# ----------------------------------

# This script obtains the rotation matrix (Q), and refernce point (ref_utm and ref_2D) from procrutes_analysis.py.
# This script read the data from car_client (lat/long in degrees and name tags). This lat/long is first converted to utm format via pyproj
# and then to 2D map coordinates using Q, ref_utm and ref_2D. 
# These 2D coordinates are then published as PeopleStamped.msg and PoseArray.msg.
# Note: The script will wait <kickout_time> seconds for user on car_client to update its position. If the script doesn't receive an update in <kickout_time>
# seconds, then the user will be deleted from peopleStamped msg array and pose array.


from std_msgs.msg import String
import numpy as np
import rospy
from pyproj import Proj
from people_msgs.msg import PersonStamped
#from people_msgs.msg import People
from rasberry_people_perception import procrutes_analysis
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from bayes_people_tracker.msg import PeopleStamped

peoplestamped = PeopleStamped()
posearray = PoseArray()
people_list = []
kickout_time=10
scaling_data = procrutes_analysis.PA()                 # Obtain returned values from procrutes_analysis's function PA
Q = scaling_data[0]      # Q is the rotation matrix
ref_utm = scaling_data[1] 
ref_2D = scaling_data[2]


class myclass(object):
    def __init__(self,gps_topic):
        self.gps_topic = gps_topic
        self.pub = rospy.Publisher('gps_positions',PeopleStamped,queue_size=10)
        self.pub2 = rospy.Publisher('gps_posearray',PoseArray,queue_size=10)
        rospy.Subscriber(gps_topic,String,self.get_locations)
        rospy.Timer(rospy.Duration(kickout_time), self.gps_active, oneshot=False)  #gps_active will be called after every <kickout_time> seconds

    def get_locations(self,msg):
        msg_data = eval(msg.data)
        name = msg_data["user"]
        lat = msg_data["lat"]
        lon = msg_data["long"]
        gac = msg_data["accu"]
        gts = str(msg_data["timestamp"])
        gts_secs = int(gts[:10])
#        gts_msecs = int(gts[10:])    
        now = rospy.get_rostime()
        myProj = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")   #Covert Lat/Lon to utm format
        x_map, y_map = myProj(lon, lat)                     #northing and easting in utm frame
        loc = np.matrix((x_map,y_map))
        loc = np.transpose(loc) 
        location = Q * (loc - ref_utm) + ref_2D             #Covert from utm frame to 2Dmap frame
        now = rospy.get_rostime()
        peoplestamped.header.frame_id = '/map'
        peoplestamped.header.stamp = now
        posearray.header.frame_id = '/map'
        posearray.header.stamp=now
        if name not in people_list:
            people_list.append(name)
            ### Populating People message ####
            self.personstamp=PersonStamped()   
            self.personstamp.header.stamp.secs = gts_secs
            self.personstamp.header.frame_id = '/map'
            self.personstamp.person.name = name
            self.personstamp.person.position.x = location[0]
            self.personstamp.person.position.y = location[1]
            self.personstamp.person.position.z = 1
            self.personstamp.person.reliability = gac
            peoplestamped.people.append(self.personstamp)
            ### Populating PosArray message ####
            self.pose=Pose()   
            self.pose.position.x = location[0]
            self.pose.position.y = location[1]
            self.pose.position.z = 0
            posearray.poses.append(self.pose)
        elif name in people_list:
            idx=people_list.index(name)
            peoplestamped.people[idx].person.position.x = location[0]
            peoplestamped.people[idx].person.position.y = location[1]
            peoplestamped.people[idx].person.position.z = 1
            peoplestamped.people[idx].header.stamp.secs = gts_secs
            peoplestamped.people[idx].person.reliability = gac
            posearray.poses[idx].position.x = location[0]
            posearray.poses[idx].position.y = location[1]
            posearray.poses[idx].position.z = 0
        else:
            pass
        self.pub.publish(peoplestamped)
        self.pub2.publish(posearray)

    def gps_active(self,mydata):                    #check for active gps 
        num_of_users = len(people_list)             #Number of active gps users
        current_time=rospy.get_rostime()            
        for i in range (num_of_users-1,-1,-1):       #range(start,stop,step_size) &  # It is necessary to run this loop in reverse direction to avoid errors as popping the first GPS from array will decrement the indices of all other GPS's.
            if len(people_list)>0 and (current_time.secs-peoplestamped.people[i].header.stamp.secs)>kickout_time:
                peoplestamped.people.pop(i)
                posearray.poses.pop(i)
                people_list.pop(i)

if __name__ == '__main__':
    rospy.init_node('gps_localization',anonymous=True)
    sml=myclass("/car_client/get_gps")
    rospy.spin()




