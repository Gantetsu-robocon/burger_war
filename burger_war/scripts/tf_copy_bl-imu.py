#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped

# copy model tf for ekf

if __name__ == '__main__':
    rospy.init_node('copy_tf_bl_imu')
    br3 = tf2_ros.StaticTransformBroadcaster()
    r = rospy.Rate(1.0)

    
    t3 = TransformStamped()
    t3.header.frame_id = 'base_link_2'
    t3.child_frame_id = 'imu_link_2'
    t3.transform.translation.x = 0.032
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 0.068
    t3.transform.rotation.x = 0.0
    t3.transform.rotation.y = 0.0
    t3.transform.rotation.z = 0.0
    t3.transform.rotation.w = 1.0

    br3.sendTransform(t3)
    rospy.spin()
