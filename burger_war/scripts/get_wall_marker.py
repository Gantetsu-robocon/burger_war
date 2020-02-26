 
#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion\
, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi

class GetMarkerInOrder():
    def __init__(self):
        rospy.init_node('wp_navi')

        # for shutdown
        rospy.on_shutdown(self.shutdown)

        # Generate action client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Wait untill actionserver starts
        while not self.ac.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")

        rospy.loginfo("The server comes up")

        # Generate Goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        way_point = [[-2.0, 3.0,-0.5 * pi], [ 3.0, 3.0, 0.0 * pi], [ 3.0,-4.5, 0.5 * pi],[ 0.0,-4.5, 1.0 * pi], [0.0, 0.0, 0.0 * pi], [999, 999, 999]]

        # Follow way point in order
        i = 0
        while not rospy.is_shutdown():
            self.goal.target_pose.pose.position.x =  way_point[i][0]
            self.goal.target_pose.pose.position.y =  way_point[i][1]

            if way_point[i][0] == 999:
                break

            q = tf.transformations.quaternion_from_euler(0, 0, way_point[i][2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            rospy.loginfo("Sending goal: No" + str(i+1))

            # Send goal to server
            self.ac.send_goal(self.goal)

            # Wait result
            succeeded = self.ac.wait_for_result(rospy.Duration(30))
            state = self.ac.get_state()
            if succeeded:
                rospy.loginfo("Succeeded: No."+str(i+1)+"("+str(state)+")")
            else:
                rospy.loginfo("Failed: No."+str(i+1)+"("+str(state)+")")

            i = i + 1

    def shutdown(self):
        rospy.loginfo("The robot was terminated")
        self.ac.cancel_goal()

if __name__ == '__main__':
    try:
        GetMarkerInOrder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")