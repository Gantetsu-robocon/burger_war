#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.
by Takuya Yamaguchi @dashimaki360
'''

import rospy
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import numpy as np
from aruco_msgs.msg import MarkerArray
import requests
import json
from time import sleep

class EnemyBot(object):
    def __init__(self, use_camera=False):
        # カメラ画像上での赤マーカ位置,サイズ
        self.cam_Point_x = 0.0
        self.cam_Point_y = 0.0
        self.cam_Point_size = 0.0
        # 相手との相対位置
        self.Relative_Pose_x = 0.0
        self.Relative_Pose_y = 0.0

        # カメラ画像上でのARマーカ位置,サイズ
        self.cam_AR_x = 0.0
        self.cam_AR_y = 0.0
        self.AR_ID = 0
        self.cam_AR_size = 0.0
        # 相手の向き（ARより）
        self.AngleEnemy_AR = 0.0
        # 緑マーカ
        self.GreenCenter_X = 0
        self.GreenCenter_Y = 0
        self.Green_Size_w = 0
        self.Green_Size_h = 0
        self.GreenSize = 0.0
        self.Green_x = 0.0

        # 青マーカ
        self.BlueCenter_X = 0
        self.BlueCenter_Y = 0
        self.Blue_Size_w = 0
        self.Blue_Size_h = 0
        self.BlueSize = 0.0

        self.real_target_id = 0

        # 相対位置座標 publisher
        self.relative_pose_pub = rospy.Publisher('relative_pose', PoseStamped ,queue_size=10)
        
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        self.color_flag_pub = rospy.Publisher('color_flag', Int8MultiArray, queue_size=10)


        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.target_id_sub = rospy.Subscriber('target_id', MarkerArray, self.targetIdCallback)
            #self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            temp_img = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.image_sub= cv2.resize(temp_img, dsize=None, fx=0.7, fy=0.7, interpolation=cv2.INTER_NEAREST)
            
    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(20)

        while not rospy.is_shutdown():

            # update PoseStamped
            temp_pose = PoseStamped()
            pose = PoseStamped()
            temp_pose.pose.position.y = self.Relative_Pose_y
            temp_pose.pose.position.x = self.Relative_Pose_x
            #print(pose.pose.position.x , pose.pose.position.y)
            euler_z = 0 # dummy
            #Gazebo座標からRviz座標
            pose.pose.position.x = temp_pose.pose.position.y
            pose.pose.position.y = -temp_pose.pose.position.x

            q = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_z)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            # update twist
            twist = Twist()
            

            #BlueマーカへのVF VF of A
#            if self.AR_ID == 0:
#                twist.linear.x = 0.1
#                twist.angular.z = (320-self.BlueCenter_X) * 0.2 / 320
#            elif self.AR_ID > 0:
#                twist.linear.x = 0.0
#                twist.angular.z = 0.0

            #GreenマーカへのVF VF of B
#            if self.AR_ID == 0:
#                twist.linear.x = 0.1
#                twist.angular.z = (320-self.GreenCenter_X) * 0.2 / 320
#            elif self.AR_ID > 0:
 #               twist.linear.x = 0.0
 #               twist.angular.z = 0.0

            # 敵が近いときのVF VF of C
 #           if self.cam_AR_size>4000 or self.GreenSize>45000: #近すぎるから離れよう
 #               twist.linear.x = -0.1
 #           else :
  #              twist.linear.x = 0.0
  #          if self.AR_ID > 0:#相手に背を向けないように動こう
  #              twist.angular.z = self.AngleEnemy_AR*0.15/(180*3.141592/180)
  #          elif self.AR_ID == 0:
  #              twist.angular.z = 0.0
            
            # publish twist topic
            self.vel_pub.publish(twist)
            # publish twist topic
            self.relative_pose_pub.publish(pose)

            self.ColorFlag = []
            if self.cam_Point_size > 0:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.BlueSize > 0:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.GreenSize > 0:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.AR_ID == 50 or self.AR_ID == 51 or self.AR_ID == 52:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)
            if self.AR_ID > 0 and self.AR_ID < 50:
                self.ColorFlag.append(1)
            else :
                self.ColorFlag.append(0)

            

            ColorFlag_forPublish = Int8MultiArray(data=self.ColorFlag)
            self.color_flag_pub.publish(ColorFlag_forPublish)

            r.sleep()

    def targetIdCallback(self, data):
        markers = data.markers
        for marker in markers:
            self.real_target_id = str(marker.id)
            #print(self.real_target_id)

    def ColorCenter(self):
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        #hsv_img = cv2.rectangle(hsv_img, (0,360), (640,480), (0,0,0), -1)
        #color_min = np.array([0,100,150])
        #color_max = np.array([50,255,255])
        #color_mask = cv2.inRange(hsv_img, color_min, color_max)

        # 赤色のHSVの値域1
        hsv_min = np.array([0,64,0])
        hsv_max = np.array([30,255,255])
        mask1 = cv2.inRange(hsv_img, hsv_min, hsv_max)
        # 赤色のHSVの値域2
        hsv_min = np.array([150,64,0])
        hsv_max = np.array([179,255,255])
        mask2 = cv2.inRange(hsv_img, hsv_min, hsv_max)
        color_mask = mask1 + mask2

        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0)
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
            if size > size_max and center_y<240:
                size_max_x = x
                size_max_y = y
                size_max_w = w
                size_max_h = h
                size_max = size
    #            center_max_x = size_max_x + size_max_w/2
    #            center_max_y = size_max_y + size_max_h/2
                center_max_x = center_x
                center_max_y = center_y

        if size_max < 100:
            
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0

        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        
        rela_pose_y= 0.0047*center_max_y + 0.4775
        rela_pose_x = ((-1.4104*rela_pose_y - 0.1011)*(340-center_max_x) +(21.627*rela_pose_y+7.827)) / 1000
        
        
        return (center_max_x, center_max_y, size_max ,rela_pose_x,rela_pose_y )

    def GreenColor(self):
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([30,64,150])
        color_max = np.array([90,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0,0.0)
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

        if size_max < 100:
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0
        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        

        return (center_max_x,center_max_y,size_max_w, size_max_h, size_max,size_max_x)


    def BlueColor(self):
        
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        color_min = np.array([90,64,0])
        color_max = np.array([150,255,255])
        color_mask = cv2.inRange(hsv_img, color_min, color_max)
        bin_img = cv2.bitwise_and(self.img, self.img, mask = color_mask)
        bin_img = cv2.cvtColor(bin_img, cv2.COLOR_BGR2GRAY)
        nLabels, label_img, data, center = cv2.connectedComponentsWithStats(bin_img)
        if nLabels < 2:
            return (0.0,0.0,0.0,0.0,0.0)
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

        if size_max < 100:
            size_max = 0
            size_max_x = 0
            size_max_y = 0
            size_max_w = 0
            size_max_h = 0
            center_max_x = 0
            center_max_y = 0

        self.img = cv2.rectangle(self.img, (size_max_x, size_max_y), (size_max_x+size_max_w, size_max_y+size_max_h), (0, 0, 0), 3)        
        #print(center_max_x)
        return (center_max_x,center_max_y,size_max_w, size_max_h, size_max)

    def ARPointSearch(self):
        if self.real_target_id == 0:
            return (0,0,0,0,0)

        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(self.img, dictionary)
        #aruco.drawDetectedMarkers(self.img, corners, ids, (0,255,0))
        
        if not corners:
            return (0,0,0,0,0)

        ARsize_max = 0
        ARcenter_max_x = 0
        ARcenter_max_y = 0
        ARsize = 0
        now_ID = 0
        for i in range(0, len(ids)):
            ARsize = (corners[i][0][1][0]-corners[i][0][0][0])*(corners[i][0][2][1]-corners[i][0][1][1])
            if ARsize_max < ARsize:
                if ARsize > 100:
                    ARcenter_max_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0])/4
                    ARcenter_max_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1])/4
                    ARsize_max = ARsize
                    now_ID = ids[i][0]
        enemy_angle = 0.0
        green_size = 0.0
        cv2.putText(self.img,str(now_ID),(70,100),cv2.FONT_HERSHEY_SIMPLEX, 3.0,(0, 0, 0),5)
        # 敵の向きを推定
        if now_ID == 50 or now_ID == 51 or now_ID == 52:
            green_w_center_x , green_center_y ,green_wx , green_wy , green_size , green_x = self.GreenColor()          
            PM_Flag = green_w_center_x - ARcenter_max_x
            #print(PM_Flag)
            if PM_Flag<-10:
                green_wx = (ARcenter_max_x - green_x)*2
            elif PM_Flag>10:
                green_wx = (green_x + green_wx - ARcenter_max_x)*2

            ttemp_theta = 0.0
            ttemp_theta = float(green_wx) / float(green_wy)
            temp_theta = -2.0964*ttemp_theta*ttemp_theta + 1.4861*ttemp_theta + 0.6648

            if now_ID == 50:
                if PM_Flag > 0:
                    enemy_angle = 90*3.141592/180 + temp_theta 
                else:
                    enemy_angle = 90*3.141592/180 - temp_theta 
            elif now_ID == 51:
                if PM_Flag > 0:
                    enemy_angle = 270*3.141592/180 + temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180
                else:
                    enemy_angle = 270*3.141592/180 - temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180

            elif now_ID == 52:
                if PM_Flag > 0:
                    enemy_angle = 180*3.141592/180 + temp_theta 
                    enemy_angle = enemy_angle - 360*3.141592/180
                else:
                    enemy_angle = 180*3.141592/180 - temp_theta                 
        self.real_target_id = 0
        return (ARcenter_max_x,ARcenter_max_y,ARsize_max,now_ID,enemy_angle)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # 敵の赤マーカ探索
        self.cam_Point_x , self.cam_Point_y , self.cam_Point_size , self.Relative_Pose_x , self.Relative_Pose_y = self.ColorCenter()
        # ARマーカ位置探索
        self.cam_AR_x , self.cam_AR_y , self.cam_AR_size , self.AR_ID , self.AngleEnemy_AR= self.ARPointSearch()
        self.GreenCenter_X , self.GreenCenter_Y , self.Green_Size_w , self.Green_Size_h , self.GreenSize , self.Green_x = self.GreenColor()
        self.BlueCenter_X , self.BlueCenter_Y , self.Blue_Size_w , self.Blue_Size_h , self.BlueSize = self.BlueColor()
        #print(self.AngleEnemy_AR*180/3.141592)
        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('relative_enemy_camera')
    bot = EnemyBot(use_camera=True)
    bot.strategy()