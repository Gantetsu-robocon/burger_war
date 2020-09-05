#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from burger_war.srv import DesiredPose, DesiredPoseResponse

class move():
    def __init__(self):
        # Initialize
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()

        self.send_goal = False
        self.succeeded = False
        self.goal = MoveBaseGoal()
        #self.status = 0
        
        # Service
        self.desired_pose_srv = rospy.Service("desired_pose", DesiredPose, self.desiredPoseCallback)
        self.reset_pathplan_sub = rospy.Service('reset_pathplan', Empty, self.resetPathplanCallback)
        rospy.wait_for_service("pathplan_succeeded")
        self.service_call = rospy.ServiceProxy("pathplan_succeeded", Empty)

    def into_field(self,goal):
        XY_LIMIT = 1.0

        ret_goal = goal

        x = ret_goal.pose.position.x
        y = ret_goal.pose.position.y

        cos45 = math.cos(math.radians(45))
        sin45 = math.sin(math.radians(45))
        x_rotate = x*cos45 + y*sin45
        y_rotate = -x*sin45 + y*cos45
        if abs(x_rotate) >= XY_LIMIT:
            x_rotate = XY_LIMIT * x_rotate / abs(x_rotate)
        if abs(y_rotate) >= XY_LIMIT:
            y_rotate = XY_LIMIT * y_rotate / abs(y_rotate)
        
        ret_goal.pose.position.x = x_rotate*cos45 - y_rotate*sin45
        ret_goal.pose.position.y = x_rotate*sin45 + y_rotate*cos45

        return ret_goal

    def sendDesiredPose(self):
        if not self.send_goal:
            return
        
        #self.ac.cancel_all_goals()
        self.ac.send_goal(self.goal)
        self.succeeded = self.ac.wait_for_result(rospy.Duration(10))

        if self.succeeded:
            try:
                self.service_call()
                self.send_goal = False
            except rospy.ServiceException, e:
                print ("Service call failed: %s" % e)
            return
    
    def desiredPoseCallback(self, data):
        self.ac.cancel_all_goals()
        self.goal.target_pose = self.into_field(data.goal)
        self.send_goal = True
        self.succeeded = False
        return DesiredPoseResponse()

    def resetPathplanCallback(self, data):
        self.ac.cancel_all_goals()
        self.send_goal = False
        self.succeeded = False
        return EmptyResponse()

    #def done_cb(self,status, result):
        #print "status:",status
        #print "result:",result
        #if status == 3:
            #print "Goal reached"
            #try:
                #self.service_call()
            #except rospy.ServiceException, e:
                #print ("Service call failed: %s" % e)
            #self.send_goal = False

    #def movebaseCallback(self, data):
        #if len(data.status_list) > 0:
            #self.status = data.status_list[0].status
            #print "self.stauts:",self.status
    
if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    mymove = move()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        mymove.sendDesiredPose()
        r.sleep()