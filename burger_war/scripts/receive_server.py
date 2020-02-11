#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json

class ServerReceiver(object):
    def __init__(self):
        # Server subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
    def serverCallback(self, data):
        print "Server Data:",data.data
        json_data = json.loads(data.data)
        #get b's score
        print "b'Score:",json_data["scores"]["b"]

    def wait(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('server_receiver')
    receiver = ServerReceiver()
    receiver.wait()


