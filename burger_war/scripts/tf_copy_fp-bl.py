#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped

# copy model tf for ekf

if __name__ == '__main__':
    rospy.init_node('copy_tf_fp_bl')
    br1 = tf2_ros.StaticTransformBroadcaster()
    r = rospy.Rate(1.0)

    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = 'base_footprint_2'
    t1.child_frame_id = 'base_link'
    t1.transform.translation.x = 0.0
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.01
    t1.transform.rotation.x = 0.0
    t1.transform.rotation.y = 0.0
    t1.transform.rotation.z = 0.0
    t1.transform.rotation.w = 1.0

    br1.sendTransform(t1)
    rospy.spin()

