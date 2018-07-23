#! /usr/bin/env python
import sys
import rospy
# Brings in the SimpleActionClient
import actionlib
import rosbag
import topological_navigation.msg

node_name = "topoNav_testcase"

#Each line of this matrix represents a row in the greenhouse, each element of a line represents a waypoint
#Matrix for Riseholme tmap
ROWS = [['WayPoint8','WayPoint20','WayPoint36','WayPoint37','WayPoint38','WayPoint39'],
        ['WayPoint7','WayPoint21','WayPoint31','WayPoint34','WayPoint32','WayPoint35'],
        ['WayPoint6','WayPoint22','WayPoint29','WayPoint30','WayPoint33','WayPoint28'],
        ['WayPoint5','WayPoint25','Waypoint23','WayPoint27','WayPoint24','WayPoint26'],
        ['WayPoint1','WayPoint41','Waypoint42','WayPoint43','WayPoint44','WayPoint45'],
        ['WayPoint2','WayPoint47','Waypoint48','WayPoint49','WayPoint50','WayPoint59'],
        ['WayPoint3','WayPoint46','Waypoint51','WayPoint52','WayPoint53','WayPoint58'],
        ['WayPoint4','WayPoint40','Waypoint54','WayPoint56','WayPoint55','WayPoint57']]
#Matrix for Norway's polytunnel tmap



class topol_nav_client:

    def __init__(self) :

        rospy.init_node(node_name)

        rospy.on_shutdown(self._on_node_shutdown)

        self.client = actionlib.SimpleActionClient('topological_navigation', topological_navigation.msg.GotoNodeAction)

        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")

    def goToWayPoint(self, waypoint):

        navgoal = topological_navigation.msg.GotoNodeGoal()

        print "Requesting Navigation to %s" %waypoint

        navgoal.target = waypoint

        # Sends the goal to the action server.
        self.client.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)

        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        # Prints out the result of executing the action
        ps = self.client.get_result()
        print ps

        """if ps.success == False
            sys.exit(1)"""


    #The robot goes straight in the row
    def goStraight(self, waypoint_list):

        for waypoint in waypoint_list:

            navigator.goToWayPoint(waypoint)


    #The robot goes forward and when it reached the end of the row it goes back
    def roundTrip(self, waypoint_list):

        path = []

        for waypoint in waypoint_list:

            path.append(waypoint)
            navigator.goToWayPoint(waypoint)

        print "End of row reached, going backward now ..."
        path.reverse()

        for waypoint in path:

            navigator.goToWayPoint(waypoint)

    #The robot navigates in all the rows
    def navigate(self):

        for i in range(len(ROWS)):

            if i%2==0:

                navigator.goStraight(ROWS[i])

            else:

                path = []

                for j in range(len(ROWS[i])-1,-1,-1):

                    path.append(ROWS[i][j])

                navigator.goStraight(path)

                path = []

        print 'Endpoint reached'



    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)

if __name__ == '__main__':


    if rospy.get_param('/testcases/task') not in range (1,4) or rospy.get_param('/testcases/rowNumber') > len(ROWS):
        print 'Usage: [1] task : (1) goStraight / (2) roundTrip / (3) navigate through all rows \n [2] index of the row for task 1 and 2 (from 0 to %d)' %(len(ROWS)-1)
        sys.exit(2)

    """if len(sys.argv) < 3 or int(sys.argv[1]) > 3 or int(sys.argv[1]) < 1 or int(sys.argv[2]) > len(ROWS):
        print 'Usage: [1] task : (1) goStraight / (2) roundTrip / (3) navigate through all rows \n [2] index of the row for task 1 and 2 (from 0 to %d)' %(len(ROWS)-1)
        sys.exit(2)"""

    navigator = topol_nav_client()

    task = rospy.get_param('/navigation_testcase/task')
    row_index = rospy.get_param('/navigation_testcase/rowNumber')

    if task == 1:

        navigator.goStraight(ROWS[row_index])

    elif task == 2:

        navigator.roundTrip(ROWS[row_index])

    elif task == 3:

        navigator.navigate()
