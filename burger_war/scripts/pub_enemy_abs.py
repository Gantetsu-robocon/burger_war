#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Get enemy absolute position from TF and publish enemy position as absolute_pose

import roslib
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped 

class PubEnemyPose():

    def __init__(self):
        self.enemy_ps = PoseStamped()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.enemy_pub = rospy.Publisher('abs_enemy_pos', PoseStamped, queue_size=10)

    def lis_pub_enemy_pose(self):
        rate = rospy.Rate(5)
        msg = PoseStamped()
        while not rospy.is_shutdown():
            try:
                t = self.tfBuffer.lookup_transform('enemy_pos', 'map', rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Eroor !')
                rate.sleep()
                continue
            self.enemy_ps.pose.position = t.transform.translation
            self.enemy_ps.pose.orientation = t.transform.rotation
            msg.pose = self.enemy_ps.pose
            self.enemy_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pub_enemy_abs')
    pub_enemy = PubEnemyPose()
    try:
        pub_enemy.lis_pub_enemy_pose()

    except rospy.ROSInterruptException: pass

