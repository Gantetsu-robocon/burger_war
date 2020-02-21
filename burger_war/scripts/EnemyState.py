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
import tf
import numpy as np

class EnemyBot(object):
    def __init__(self, use_camera=False):
        # カメラ画像上での赤マーカ位置,サイズ
        self.cam_Point_x = 0.0
        self.cam_Point_y = 0.0
        self.cam_Point_size = 0.0
        # velocity publisher
        self.relative_pose_pub = rospy.Publisher('relative_pose', PoseStamped ,queue_size=10)

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
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            euler_z = 0 # dummy

            q = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_z)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # publish twist topic
            self.relative_pose_pub.publish(pose)

            r.sleep()

    def GetThresholdedImage(self,imgHSV):
        imgResult = cv.CreateImage(cv.GetSize(imgHSV), cv.IPL_DEPTH_8U, 1)
        cv.InRangeS(imgHSV, cv.Scalar(150, 160, 60), cv.Scalar(200, 256, 256), imgResult)
        return imgResult

    def ColorCenter(self):
#        self.img = cv2.rectangle(self.img, (0,360), (640,480), (0,0,0), -1)
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([0,100,150])
        color_max = np.array([50,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        #print(nLabels)
        if nLabels < 2:
            return (0.0,0.0,0.0)
        size_max = 0
        size_max_x = 0
        size_max_y = 0
        size_max_w = 0
        size_max_h = 0
        center_max_x = 0
        center_max_y = 0

        for i in range(1, nLabels):
            x, y, w, h, size = data[i]
            center_x, center_y = center[i]
            if size > size_max:
                size_max_x = x
                size_max_y = y
                size_max_w = w
                size_max_h = h
                size_max = size
    #            center_max_x = size_max_x + size_max_w/2
    #            center_max_y = size_max_y + size_max_h/2
                center_max_x = center_x
                center_max_y = center_y

        print(size_max)
        if size_max < 30:
            center_max_x = 0
            center_max_y = 0
            size_max = 0
        print(size_max)
        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 255, 255), 3)        

        return (center_max_x, center_max_y, size_max)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # 色重心
        self.cam_Point_x , self.cam_Point_y , self.cam_Point_size = self.ColorCenter()
        print(self.cam_Point_x , self.cam_Point_y , self.cam_Point_size)
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('relative_enemy_camera')
    bot = EnemyBot(use_camera=True)
    bot.strategy()

