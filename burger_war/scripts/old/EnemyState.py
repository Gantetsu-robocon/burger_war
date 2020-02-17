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
    def __init__(self,use_camera=False):



        # velocity publisher
        self.relative_pose_pub = rospy.Publisher('relative_pose', PoseStamped ,queue_size=1)

         # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        

    def GetThresholdedImage(imgHSV):
        imgResult = cv.CreateImage(cv.GetSize(imgHSV), cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(imgHSV, cv.Scalar(150, 160, 60), cv.Scalar(200, 256, 256), imgResult)
        return imgResult

    def ColorCenter(Image):
        # Cの色重心プログラム(return list)
        capture = None
        frame = None
        imgR = imgG = imgB = None
        imgThreshold_R = imgThreshold_G = imgThreshold_B = None
        imgTmp = imgHSV = imgResult = None
        moments = None
        OP = [0,0]

        frame = Image
        frame = cv.CloneImage(frame)
        cv.Smooth(frame, frame, cv.CV_GAUSSIAN, 3, 3) 
        the_size = cv.GetSize(frame)

        imgR = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgG = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgB = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgThreshold_R = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgThreshold_G = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgThreshold_B = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgResult = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
        imgTmp = cv.CreateImage(the_size, cv.IPL_DEPTH_8U, 1)
            
        cv.Split(frame, imgB, imgG, imgR, None) # BGRを分解
            
        # 赤の要素が100以上で、緑と青より1.5倍以上あるピクセルを抽出
        cv.Threshold(imgR, imgThreshold_R, 100, 255, cv.CV_THRESH_BINARY)
        cv.Div(imgR, imgG, imgTmp, 10) # 10倍
        cv.Threshold(imgTmp, imgThreshold_G, 15, 255, cv.CV_THRESH_BINARY)
        cv.Div(imgR, imgB, imgTmp, 10)
        cv.Threshold(imgTmp, imgThreshold_B, 15, 255, cv.CV_THRESH_BINARY)
        cv.And(imgThreshold_G, imgThreshold_B, imgTmp, None)
        cv.And(imgTmp, imgThreshold_R, imgResult, None)

        # cv.Moments(imgResult, moments, 0)
        moments = cv.Moments(cv.GetMat(imgResult, 1), 0)
        m00 = cv.GetSpatialMoment(moments, 0, 0)
        m10 = cv.GetSpatialMoment(moments, 1, 0)
        m01 = cv.GetSpatialMoment(moments, 0, 1)
        if (m00 != 0):
            gX = int(m10 // m00)
            gY = int(m01 // m00)
            # cvCircle(frame, cvPoint(gX, gY), 80, CV_RGB(0,0,255), 6, 8, 0);
            cv.Circle(frame, (gX, gY), 80, (0, 0, 255), 6, 8, 0)
        else:
            # Color Detection & Object Tracking
            # http://opencv-srf.blogspot.ro/2010/09/object-detection-using-color-seperation.html
            imgHSV = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(frame, imgHSV, cv.CV_BGR2HSV) # Change the color format from BGR to HSV
            imgResult = GetThresholdedImage(imgHSV)
            cv.Smooth(imgResult, imgResult, cv.CV_GAUSSIAN, 3, 3) # smooth the binary image using Gaussian kernel
        cv2.imshow("Results", imgResult)
        cv2.imshow("Video", frame)

        OP = [gX,gY]

        return OP

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        center_of_image = [240,180]
        CP = [0,0]
        # 色重心        
        CP = self.ColorCenter(self.img)

        # 各値計算
        pose = PoseStamped()
        pose.pose.position.x = CP[1] * K2
        pose.pose.position.y = (CP[0] - center_of_image[0]) * K1
        pose.pose.orientation.z = 0

        self.relative_pose_pub.publish(pose)

        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('enemystate')
    bot = EnemyBot(use_camera=True)
    ##bot.strategy()
