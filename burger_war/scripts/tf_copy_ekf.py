#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped

# copy model tf for ekf

if __name__ == '__main__':
    rospy.init_node('copy_tf_broadcaster')
    br = tf2_ros.StaticTransformBroadcaster()
    r = rospy.Rate(1.0)

    
    t = TransformStamped()
    t.header.frame_id
    t.child_frame_id
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)
    rospy.spin()
