import rospy
import rospkg
import yaml
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.srv import ReconfigureRequest, Reconfigure
from dynamic_reconfigure.msg import DoubleParameter, BoolParameter

from std_msgs.msg import Bool

class TopicPublisherPolicy:
    def __init__(self):
        self._topic_pub = rospy.Publisher("lock_all", Bool, queue_size=1)

    def execute(self):
        rospy.logerr("Stopping")
        self._topic_pub.publish(Bool(data = True))

    def stop(self):
        rospy.logwarn("Release Stop")
        self._topic_pub.publish(Bool(data = False))


#This action sometime hangs
class DynamicReconfigurePolicy:
    def __init__(self, cfg_package="rasberry_han", cfg_folder = "/config/"):
        self._safe_path = rospkg.RosPack().get_path(cfg_package) + cfg_folder
        self._param_server = yaml.load(open(self._safe_path + "proximity_params.yaml"))
        self._dynamic_reconfigure_client = Client("/move_base/DWAPlannerROS", timeout=4)

    def execute(self):
        rospy.logwarn("Slowdown")
        self._dynamic_reconfigure_client.update_configuration(self._param_server['safe_params'])

    def stop(self):
        rospy.logwarn("Reconfigure Default")
        self._dynamic_reconfigure_client.update_configuration(self._param_server['default_params'])


class DynamicReconfigureServicePolicy:
    def __init__(self, cfg_package="rasberry_han", cfg_folder = "/config/"):
        self._safe_path = rospkg.RosPack().get_path(cfg_package) + cfg_folder
        self._param_server = yaml.load(open(self._safe_path + "proximity_params.yaml"))
        rospy.logwarn("waiting service")
        rospy.wait_for_service('/move_base/DWAPlannerROS/set_parameters')
        rospy.logwarn("service received")
        self._service_client = rospy.ServiceProxy('/move_base/DWAPlannerROS/set_parameters', Reconfigure)

    def execute(self):
        rospy.logwarn("Slowdown")
        request = ReconfigureRequest()
        params = self._param_server['safe_params']

        #assume all double
        for key, value in params.iteritems():
            p = DoubleParameter()
            p.name = key
            p.value = value
            request.config.doubles.append(p)

        self._service_client(request)


    def stop(self):
        rospy.logwarn("Reconfigure Default")
        request = ReconfigureRequest()
        params = self._param_server['default_params']

        #assume all bool
        for key, value in params.iteritems():
            p = BoolParameter()
            p.name = key
            p.value = value
            request.config.bools.append(p)

        self._service_client(request)
