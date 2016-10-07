#!/usr/bin/env python
import roslib
roslib.load_manifest("wifi_localization")
import rospy
from subprocess import call, PIPE, STDOUT, Popen
from wifi import Wifi
import re
from wifi_localization.msg import WifiState

class WifiPublisher:
    def __init__(self):
        rospy.on_shutdown(self.exit)
        rospy.init_node("wifi_state")
        self.publisher = rospy.Publisher('wifi_state', WifiState, queue_size=0)
        rospy.loginfo("Initialized wifi_state publisher")
        self.macfilter = re.compile(r'([0-9A-F]{2}[:-]){5}([0-9A-F]{2})', re.I)
        self.state = {}
        self.run()

    def run(self):
        wifi_interface = rospy.get_param("wifi_publisher/wifi_interface","wlan0")
        while not rospy.is_shutdown():
            proc = Popen("sudo iw dev "+wifi_interface+" scan | grep -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}|SSID|signal'", shell=True, stdout=PIPE, stderr=STDOUT)
            proc_out, proc_err = proc.communicate()
            result = self.parse_wifi_scan(proc_out)
            time = rospy.Time.now()
            #print result
            ssids = []
            macs = []
            strengths = []
            for wifi in result:
                ssids.append(wifi.ssid)
                macs.append(wifi.mac)
                strengths.append(float(wifi.strength))
            wifistate_msg = WifiState()
            wifistate_msg.header.stamp = time
            wifistate_msg.ssids = ssids
            wifistate_msg.macs = macs
            wifistate_msg.strengths = strengths
            self.publisher.publish(wifistate_msg)

    def parse_wifi_scan(self, scan_result):
        lines = scan_result.split('\n')
        result = []
        mac = False
        strength = False
        ssid = False
        for line in lines:
            if not mac:
                if line.startswith('BSS'):
                    mac = re.search(self.macfilter, line).group(0)
            elif not strength:
                if line.lstrip().startswith('signal'):
                    strength = line.split(' ')[1]
            elif not ssid:
                if line.lstrip().startswith('SSID'):
                    ssid = line.split(' ')[1]
                    result.append(Wifi(mac, ssid, strength))
                    mac = False
                    ssid = False
                    strength = False
        return result

    def exit(self):
        rospy.logerr("Shutdown request")

if __name__ == '__main__':
    try:
        wifistate = WifiPublisher()
    except rospy.ROSInterruptException:
        pass
