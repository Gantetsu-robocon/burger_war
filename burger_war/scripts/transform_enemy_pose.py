#!/usr/bin/env python
# -*- coding: utf-8 -*-

#TF enemy position from ralative_pos topic
#Add time losed enemy to color_flag 

import rospy
import tf2_ros
import tf_conversions
import tf
import math
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8MultiArray

class TransformEnemy():

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.enemy_ps = PoseStamped()
        self.enemy_sub = rospy.Subscriber('/relative_pose', PoseStamped, self.enemyCallback)
        self.flag_sub = rospy.Subscriber('/color_flag', Int8MultiArray,self.flagCallback)
        self.flag_pub = rospy.Publisher('color_flag_time', Int8MultiArray, queue_size=10)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.position.x = 2.6
        self.enemy_ps.pose.orientation = rotation
        #self.enemy_ps_save = self.enemy_ps
        self.flags = [0, 0, 0, 0, 0, 0]
        self.initial_time = rospy.Time.now().secs

    #敵が見えてるときのみ更新
    def enemyCallback(self,data):
        if (self.flags[0] + self.flags[1] + self.flags[3]) != 0 :
            self.enemy_ps.pose.position = data.pose.position
            self.enemy_ps.pose.orientation = data.pose.orientation
        #self.enemy_ps_save = self.enemy_ps

    def flagCallback(self, data):
        for i, flag in enumerate(data.data):
            self.flags[i] = flag
        if (self.flags[0] + self.flags[1] + self.flags[3]) == 0 :
            current_time = rospy.Time.now().secs
            self.flags[5] = current_time - self.initial_time
    
    def tf_enemy_pose(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'enemy_pos'
        t.transform.translation = self.enemy_ps.pose.position
        t.transform.rotation = self.enemy_ps.pose.orientation
        self.br.sendTransform(t)
    
    def publish_flags(self):
        array_forPublish = Int8MultiArray(data=self.flags)
        self.flag_pub.publish(array_forPublish)

    def main(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.tf_enemy_pose()
            self.publish_flags()
            rate.sleep()

    
        
if __name__ == '__main__':
    rospy.init_node('enemy_tf_broadcaster')
    br_enemy = TransformEnemy()
    br_enemy.main()
    

