#!/usr/bin/env python
# -*- coding: utf-8 -*-

#TF enemy position from ralative_pos topic

import rospy
import tf2_ros
import tf_conversions
import tf
import math
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

class TransformEnemy():

    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.enemy_ps = PoseStamped()
        self.enemy_sub = rospy.Subscriber('/relative_pose', PoseStamped, self.enemyCallback)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        rotation = Quaternion(*q)
        self.enemy_ps.pose.position.x = 1.3
        self.enemy_ps.pose.orientation = rotation

    def tf_enemy_pose(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'enemy_pos'

            t.transform.translation = self.enemy_ps.pose.position
            t.transform.rotation = self.enemy_ps.pose.orientation

            self.br.sendTransform(t)
            rate.sleep()

    def enemyCallback(self,data):
        self.enemy_ps.pose.position = data.pose.position
        self.enemy_ps.pose.orientation = data.pose.orientation

if __name__ == '__main__':
    rospy.init_node('enemy_tf_broadcaster')
    br_enemy = TransformEnemy()
    br_enemy.tf_enemy_pose()
    

