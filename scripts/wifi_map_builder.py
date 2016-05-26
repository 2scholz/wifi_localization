#!/usr/bin/env python
import roslib
roslib.load_manifest("wifi_localization")
import rospy
#import socket
#import os
#import sys
#from std_msgs.msg import String
#from threading import Thread

#class MMTest:
#    def __init__(self):
#        rospy.on_shutdown(self.exit)
#        self.hostname = socket.gethostname()
#        self.ip = ([(s.connect(('8.8.8.8', 80)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1])
#        rospy.init_node("multimaster_test_%s" % self.hostname)
#        self.publisher = rospy.Publisher('/chatter', String, queue_size=0)
#        rospy.Subscriber('/chatter', String, self.callback)
#        rospy.loginfo("Initialized multimaster_test on %s" % self.hostname)
#        pub_thread = Thread(target=self.pub)
#        pub_thread.start()
#        self.run()
#
#    def run(self):
#        while not rospy.is_shutdown():
#            rospy.spin()
#
#    def pub(self):
#        rospy.sleep(2)
#        while not rospy.is_shutdown():
#            self.ip = ([(s.connect(('8.8.8.8', 80)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1])
#            self.hostname = socket.gethostname()
#            msg_str = "MMtest: [%s @ %s] %s" % (self.hostname, self.ip, rospy.get_rostime().secs)
#            self.publisher.publish(msg_str)
#            rospy.sleep(5)
#
#    def callback(self, data):
#        print "Received msg: %s" % data.data
#
#    def exit(self):
#        rospy.logerr("Shutdown request")
#
if __name__ == '__main__':
    try:
        #mmtest = MMTest()
        print 'test'
    except rospy.ROSInterruptException:
        pass
