#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Publish my position from TF 
import roslib
import rospy
import tf2_ros
import tf_conversions
import tf
import math 
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Quaternion

class PubEnemyPose():

    def __init__(self):
        self.my_ps = PoseStamped()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        rotation = Quaternion(*q)
        self.my_ps.pose.orientation = rotation

        self.my_pub = rospy.Publisher('my_pose', PoseStamped, queue_size=10)

    def lis_pub_my_ps(self):
        rate = rospy.Rate(5)
        msg = PoseStamped()
        while not rospy.is_shutdown():
            try:
                t = self.tfBuffer.lookup_transform('base_link', 'map', rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr('LookupTransform Eroor !')
                rate.sleep()
                continue
            self.my_ps.pose.position = t.transform.translation
            self.my_ps.pose.orientation = t.transform.rotation
            msg.pose = self.my_ps.pose
            self.my_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pub_my_pose')
    pub_self = PubEnemyPose()
    try:
        pub_self.lis_pub_my_ps()

    except rospy.ROSInterruptException: pass

