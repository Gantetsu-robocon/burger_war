#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.

by Takuya Yamaguchi @dashimaki360
'''

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2


class EnemyBot(object):
    def __init__(self, use_camera=False):
        # カメラ画像上での赤マーカ位置
        self.CP = [0,0]
        # velocity publisher
        self.relative_pose_pub = rospy.Publisher('relative_pose', PoseStamped ,queue_size=1)
                # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            # update PoseStamped
            pose = PoseStamped()
            pose.pose.position.x = self.CP[1]
            pose.pose.position.y = self.CP[0]
            pose.pose.orientation.z = 0

            # publish twist topic
            self.relative_pose_pub.publish(pose)

            r.sleep()

    def GetThresholdedImage(self,imgHSV):
        imgResult = cv.CreateImage(cv.GetSize(imgHSV), cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(imgHSV, cv.Scalar(150, 160, 60), cv.Scalar(200, 256, 256), imgResult)
        return imgResult

    def ColorCenter(self,Image):
        OP = [10,0]
        return OP

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # 色重心        
        self.CP = self.ColorCenter(self.img)
        
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = EnemyBot(use_camera=True)
    bot.strategy()


