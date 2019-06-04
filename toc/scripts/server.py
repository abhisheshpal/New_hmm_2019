#!/usr/bin/python

import rospy
import roslib

import web
import signal
from os import chdir, getenv
from os.path import join
from yaml import safe_load, YAMLError
import json


#from strands_executive_msgs.srv import DemandTask as SchedulerDemandTask
#from strands_executive_msgs.srv import DemandTaskRequest as SchedulerDemandTaskRequest

### Templates
TEMPLATE_DIR = roslib.packages.get_pkg_dir('toc') + '/www'
WEBTOOLS_DIR = roslib.packages.get_pkg_dir('strands_webtools')
CONFIG_FILE = roslib.packages.get_pkg_dir('toc') + '/conf/default.yaml'


html_config = {
    'rosws_suffix': ':9246',
    'mjpeg_suffix': ':8181',
    'rosws_protocol': 'ws'
}

render = web.template.render(TEMPLATE_DIR, base='base', globals=globals())
chdir(TEMPLATE_DIR)


class Config:
    class __Config:
        def __init__(self, filename):
            self.filename = filename

            with open(self.filename, 'r') as stream:
                try:
                    self._config = safe_load(stream)
                except YAMLError as exc:
                    rospy.logfatal(exc)
                    raise

    __instance = None

    def __init__(self, filename=CONFIG_FILE):
        if not Config.__instance:
            Config.__instance = Config.__Config(filename)

    def get(self):
        return self.__instance._config


class ControlServer(web.application):
    def __init__(self):
        urls = (
            '/', 'IndexPage',
            '/robot/(.*)/dashboard', 'Dashboard',
            '/webtools/(.*)', 'Webtools'
        )
        web.application.__init__(self, urls, globals())

        signal.signal(signal.SIGINT, self.signal_handler)

    def run(self, port=8027, *middleware):
        func = self.wsgifunc(*middleware)
        return web.httpserver.runsimple(func, ('0.0.0.0', port))

    def signal_handler(self, signum, frame):
        self.stop()



def set_ws_protocol():
    forward = web.ctx.env.get('HTTP_X_FORWARDED_HOST', '')
    if 'lcas.lincoln.ac.uk' in forward:
        html_config['rosws_protocol'] = 'wss'
    else:
        html_config['rosws_protocol'] = 'ws'
    print html_config['rosws_protocol']


class Dashboard(object):
    def GET(self, robot):
        set_ws_protocol()
        # send a refresh every 5 minutes, just to be sure.
        #web.header('Refresh', '300')
        try:
            c = Config().get()['robots'][robot]
            return render.dashboard(robot, json.dumps(c))
        except Exception as e:
            rospy.logerr('robot %s is not known' % robot)
            raise
            return web.notfound('robot %s is not known' % robot)


class IndexPage(object):
    def GET(self):
        set_ws_protocol()
        # send a refresh every 10 minutes, just to be sure.
        #web.header('Refresh', '600')
        return render.index(Config().get())


class Webtools(object):
    """
    proxies all requests to strands_webtools
    """
    def GET(self, f):
        try:
            p = join(WEBTOOLS_DIR, f)
            rospy.logdebug("trying to serve %s from %s", f, p)
            if f.endswith('.js'):
                web.header('Content-Type', 'text/javascript')
            return open(p, 'r').read()
        except:
            web.application.notfound(app)  # file not found


if __name__ == "__main__":
    rospy.init_node("toc_server")
    Config()
    port = rospy.get_param('~port', 8127)
    html_config['rosws_suffix'] = rospy.get_param('~rosws_suffix', ":9246")
    html_config['mjpeg_suffix'] = rospy.get_param('~mjpeg_suffix', ":8080")
    html_config['rosws_protocol'] = rospy.get_param('~rosws_protocol', "ws")

    rospy.loginfo("lindimp_control_server started.")
    app = ControlServer()
    app.run(port=port)
