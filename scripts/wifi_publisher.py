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
        self.freq_list = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472, 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700]
        self.freq_list = rospy.get_param("wifi_publisher/frequencies",self.freq_list)
        self.discard_data_thresh_ms = 50000
        self.discard_data_thresh_ms = rospy.get_param("wifi_publisher/discard_data_thresh_ms",self.discard_data_thresh_ms)
        self.state = {}
        self.run()

    def run(self):
        wifi_interface = rospy.get_param("wifi_publisher/wifi_interface","wlan0")
        while not rospy.is_shutdown():
            for freq in self.freq_list:
                proc = Popen("sudo iw dev "+wifi_interface+" scan freq "+str(freq)+" | grep -E '([[:xdigit:]]{1,2}:){5}[[:xdigit:]]{1,2}|SSID|signal|last seen|freq'", shell=True, stdout=PIPE, stderr=STDOUT)
                proc_out, proc_err = proc.communicate()
                result = self.parse_wifi_scan(proc_out)
                time = rospy.Time.now()
                #print result
                ssids = []
                macs = []
                strengths = []
                last_seen = []
                frequencies = []
                if not result:
                    continue
                for wifi in result:
                    ssids.append(wifi.ssid)
                    macs.append(wifi.mac)
                    strengths.append(float(wifi.strength))
                    last_seen.append(int(wifi.last_seen))
                    frequencies.append(int(wifi.frequency))
                wifistate_msg = WifiState()
                wifistate_msg.header.stamp = time
                wifistate_msg.ssids = ssids
                wifistate_msg.macs = macs
                wifistate_msg.frequencies = frequencies
                wifistate_msg.strengths = strengths
                wifistate_msg.last_seen = last_seen
                wifistate_msg.frequencies = frequencies
                self.publisher.publish(wifistate_msg)

    def parse_wifi_scan(self, scan_result, current_freq=0):
        lines = scan_result.split('\n')
        result = []
        mac = False
        strength = False
        ssid = False
        last_seen = False
        frequency = False
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
            elif not last_seen:
                if line.lstrip().startswith('last'):
                    last_seen = line.split(' ')[2]
            elif not frequency:
                if line.lstrip().startswith('freq'):
                    frequency = line.split(' ')[1]
                    if (int(current_freq) == int(frequency) or int(current_freq)==0) and (int(last_seen) <= self.discard_data_thresh_ms): 
                        result.append(Wifi(mac, ssid, strength, last_seen, frequency))
                    mac = False
                    ssid = False
                    strength = False
                    last_seen = False
                    frequency = False
        return result

    def exit(self):
        rospy.logerr("Shutdown request")

if __name__ == '__main__':
    try:
        wifistate = WifiPublisher()
    except rospy.ROSInterruptException:
        pass
