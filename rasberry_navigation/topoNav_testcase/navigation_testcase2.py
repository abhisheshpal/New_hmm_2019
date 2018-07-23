#! /usr/bin/env python
import sys
import rospy
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg

node_name = "topoNav_testcase"

#Each line of this matrix represents a row in the greenhouse, each element of a line represents a waypoint
ROWS = [['WayPoint8','WayPoint20','WayPoint36','WayPoint37','WayPoint38','WayPoint39'],
        ['WayPoint7','WayPoint21','WayPoint31','WayPoint34','WayPoint32','WayPoint35'],
        ['WayPoint6','WayPoint22','WayPoint29','WayPoint30','WayPoint33','WayPoint28'],
        ['WayPoint5','WayPoint25','Waypoint23','WayPoint27','WayPoint24','WayPoint26'],
        ['WayPoint1','WayPoint41','Waypoint42','WayPoint43','WayPoint44','WayPoint45'],
        ['WayPoint2','WayPoint47','Waypoint48','WayPoint49','WayPoint50','WayPoint59'],
        ['WayPoint3','WayPoint46','Waypoint51','WayPoint52','WayPoint53','WayPoint58'],
        ['WayPoint4','WayPoint40','Waypoint54','WayPoint56','WayPoint55','WayPoint57']]

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


    def goStraigth(self, row_index):

        for waypoint in ROWS[row_index]:

            navigator.goToWayPoint(waypoint)


    def roundTrip(self, row_index):

        path = []

        for waypoint in ROWS[row_index]:

            tempPath.append(waypoint)
            navigator.goToWayPoint(waypoint)

        print "End of row reached, going backward now ..."
        path.reverse()

        for waypoint in path:

            navigator.goToWayPoint(waypoint)



    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        #sleep(2)


if __name__ == '__main__':

    if len(sys.argv) < 3 or int(sys.argv[1]) != (1 or 2) or int(sys.argv[2]) > 7:
        print 'Argument required: [1] task : (1) goStraigth or (2) roundTrip \n [2] number of the row (from 0 to 7)'
        sys.exit(2)

    navigator = topol_nav_client()

    task = int(sys.argv[1])
    row_index = int(sys.argv[2])

    if task == 1:

        navigator.goStraigth(row_index)

    elif task == 2:

        navigator.roundTrip(row_index)
