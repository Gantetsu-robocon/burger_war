#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped

# copy model tf for ekf

if __name__ == '__main__':
    rospy.init_node('tf_copy_bl_ld')
    br2 = tf2_ros.StaticTransformBroadcaster()
    r = rospy.Rate(1.0)

    
    
    t2 = TransformStamped()
    t2.header.frame_id = 'base_link_2'
    t2.child_frame_id = 'base_scan'
    t2.transform.translation.x = -0.032
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.172
    t2.transform.rotation.x = 0.0
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0

    
    br2.sendTransform(t2)
    rospy.spin()
