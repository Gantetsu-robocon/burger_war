#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Get enemy absolute position from TF and publish enemy position as absolute_pose
import roslib
import rospy
import tf2_ros
import tf_conversions
import tf
import math 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Quaternion
from std_msgs.msg      import Int16MultiArray
from std_msgs.msg      import Bool 
"""
import quaternion
import numpy as np
"""
class PubEnemyPose():

    def __init__(self):

        #Get parameter
        self.rate = rospy.get_param("~rate", 1)
        #self.side = rospy.get_param("~side", "r")

        #Transformer, Listener, Subscriber, Publisher
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.enemy_pub = rospy.Publisher('absolute_pos', PoseStamped, queue_size=10)
        self.flag_sub = rospy.Subscriber('/color_flag_time', Int16MultiArray,self.flagcolorCallback)
        self.flag_sub = rospy.Subscriber('/lidar_flag', Bool, self.flaglidarCallback)

        #Initialize 
        self.enemy_ps = PoseStamped()
        self.enemy_ps.pose.position.x = 1.3
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.orientation = rotation
        self.flag_color = False
        self.flag_lidar = False
        self.t_camera = PoseStamped()
        self.t_camera.pose.orientation = (0.0, 0.0, 0.0, 1.0)
        self.t_lidar = PoseStamped()
        self.t_lidar.pose.orientation = (0.0, 0.0, 0.0, 1.0)


    def lisn_enemy_camera(self):
        try:
            t = self.tfBuffer.lookup_transform('map', 'enemy_camera_abs', rospy.Time(0), rospy.Duration(1.0))
            self.t_camera.pose.position = t.transform.translation
            self.t_camera.pose.orientation = t.transform.rotation
            self.flag_color = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.flag_color = False
    
    def lisn_enemy_lidar(self):
        try:
            t = self.tfBuffer.lookup_transform('map', 'enemy_lidar', rospy.Time(0), rospy.Duration(1.0))
            self.t_lidar.pose.position = t.transform.translation
            self.t_lidar.pose.orientation = t.transform.rotation
            self.flag_lidar = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.flag_lidar = False

    def pub_enemy_abs(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        if self.flag_lidar == True:
            msg.pose.position = self.t_lidar.pose.position
            msg.pose.orientation = self.t_lidar.pose.orientation
            self.enemy_pub.publish(msg)
            #rospy.loginfo("Publish Lidar pose")
        elif self.flag_color == True:
            msg.pose.position = self.t_camera.pose.position
            
            q = self.t_camera.pose.orientation
            msg.pose.orientation = Quaternion(q[3],q[4],q[5],q[6])
            
            #msg.pose.orientation = self.t_camera.pose.orientation
            self.enemy_pub.publish(msg)
            #rospy.loginfo("Publish Camera pose")

    def flagcolorCallback(self,flag):
        if (flag.data[0] + flag.data[2] + flag.data[3]) == 0 :
            self.flag_color = False
        else:
            self.flag_color = True

    def flaglidarCallback(self,flag):
        self.flag_lidar = flag

    def main(self):
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            self.lisn_enemy_camera()
            self.lisn_enemy_lidar()
            if (self.flag_color==True) or (self.flag_lidar==True):
                self.pub_enemy_abs()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pub_enemy')
    pub_enemy = PubEnemyPose()
    pub_enemy.main()