#!/usr/bin/env python
import rospy
import sys
from std_srvs.srv import SetBool
import serial


# A class for the Thorvald UV rig
class UVRig:

    # Some class members that we need
    _port = None
    _baud = None
    _ser_uv = None


    # Constrictor
    def __init__(self, port, baud, use_sim):


        # Advertise service
        self._srv_lights = rospy.Service('switch_uv', SetBool, self.lights_callback)

        # Port, baud and serial connection
        self._port = port
        self._baud = baud
        self._use_sim = use_sim

        if not self._use_sim:
            self._ser_uv = serial.Serial(self._port, self._baud, timeout=1)

            # Verify that we are talking to the correct device
            connected = False
            print('connecting to device..')
            while (not connected and not rospy.is_shutdown()):
                self._ser_uv.write('?ZUP\n')
                line = self._ser_uv.readline()
                if line == '~HELLO\n':
                    connected = True
            print('Connected!')

        # Spin until the end of time or until ros shuts down... whatever comes first
        rospy.spin()

        if not self._use_sim:
            # Close serial connection
            self._ser_uv.close()

    # Switch uv lights on or off
    def switch_lights(self, lights_on):
        if lights_on:
            print('Lights on')
            if not self._use_sim:
                self._ser_uv.write('!L:1\n')
        else:
            print('Lights off')
            if not self._use_sim:
                self._ser_uv.write('!L:0\n')

    # Callback for our switch uv service
    def lights_callback(self, req):
        self.switch_lights(req.data)
        return (True, "")

if __name__ == '__main__':
    rospy.init_node('uv_rig')

    if len(sys.argv) < 4:
        print("usage: uv_rig.py port baud use_sim")
        raise Exception("usage: uv_rig.py port baud use_sim")
    else:
        port = sys.argv[1]
        baud = sys.argv[2]
        use_sim = sys.argv[3].lower() == 'true'

    rospy.loginfo('Starting..')

    try:
        uv_rig = UVRig(port, baud, use_sim)
    except rospy.ROSInterruptException:
        pass
