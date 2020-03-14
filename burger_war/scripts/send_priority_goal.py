#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Import
import rospy
import json
import numpy as np
import tf
import sys
import os
import actionlib
import copy
from transitions.extensions import GraphMachine
import time

#Import ROS topic type
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose
from std_msgs.msg import String, Int16MultiArray, Int8

#Import ROS service type
from burger_war.srv import DesiredPose, VisualFeedbackFlag
from std_srvs.srv import Empty,EmptyResponse

#分割
import def_state
from server_receiver import ServerReceiver

#for debug
from IPython.terminal.debugger import set_trace
        
class SendPriorityGoal(ServerReceiver): #ServerReceiverの継承
    def __init__(self):
        super(SendPriorityGoal,self).__init__()
        #Get parameter
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.50)
        self.time_th = rospy.get_param("~time_th", 0)
        self.control_cycle = rospy.get_param("~control_cycle", 5.0)
        self.diff_theta_th = rospy.get_param("~diff_theta_th",0.7854) #pi/4
        self.close_th = rospy.get_param("~close_th",0.8)
        self.vf_A_dist = rospy.get_param("~vf_A_dist",self.focus_dist)
        self.vf_B_dist = rospy.get_param("~vf_B_dist",1.5*self.focus_dist)
        
        #Service Clint
        rospy.wait_for_service("desired_pose")
        self.desired_pose_call = rospy.ServiceProxy("desired_pose",DesiredPose)
        rospy.wait_for_service("reset_pathplan")
        self.cancel_goal_call = rospy.ServiceProxy("reset_pathplan", Empty)
        rospy.wait_for_service("vf_flag")
        self.vf_flag_call = rospy.ServiceProxy("vf_flag", VisualFeedbackFlag) 

        #Service Server
        self.path_success_srv = rospy.Service("pathplan_succeeded",Empty, self.successCallback)

    def successCallback(self,req):
        self.goal_reached = True
        return EmptyResponse()

    def send_goal(self,target_name):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.target_states[target_name]["pose"][0]
        goal.pose.position.y = self.target_states[target_name]["pose"][1]

        q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target_name]["pose"][2])
        goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        #ゴールをGlobal Plannerに送る
        self.desired_pose_call(goal)

    def stop_sending(self):
        self.cancel_goal_call()
        self.vf_flag_call(Int8(data=0))

    def vf(self,previous_state,target):
        pre_state = previous_state
        if self.color_flag[1] and self.target_distance(target) <self.vf_A_dist:
            if not pre_state == "vf_A":
                self.stop_sending()
                #vf Aスタート
                self.vf_flag_call(Int8(data=0))
                print ""
                print "vf A starts"
                print ""
                self.vf_flag_call(Int8(data=1))
                pre_state = "vf_A"

            if self.color_flag[4]:
                self.vf_flag_call(Int8(data=0))
                self.target_states[target]["priority"] = -99

        if self.color_flag[2] and self.enemy_distance() < self.vf_B_dist:
            if not pre_state == "vf_B":
                self.stop_sending()
                #vf Bスタート
                self.cancel_goal_call()
                print ""
                print "vf B starts"
                print ""
                self.vf_flag_call(Int8(data=2))
                pre_state = "vf_B"

            if self.color_flag[3]:
                self.vf_flag_call(Int8(data=0))
                self.target_states[target]["priority"] = -99

        return pre_state

    def main(self):
        pre_state = ""
        while not rospy.is_shutdown():
            self.target_priority_update()

            #敵との距離が近いか
            ene_is_near = True if self.enemy_distance() < self.enemy_distance_th else False
            ene_is_far = True if not ene_is_near else False
            #相手と向かい合っているかどうか
            ene_to_face = True if self.diff_theta < self.diff_theta_th else False
            ene_not_face = True if not ene_to_face else False
            #相手が見えているかどうか
            find_enemy = True if self.color_flag[2] else False
            lose_enemy = True if not find_enemy else False
            #時間が残っているか
            in_time = True if self.passed_time < self.time_th else False
            over_time = True if in_time else False
            #かなり近いところに壁のマーカーがあるか
            _, nearest_dist = self.nearest_target()
            marker_is_close = True if nearest_dist < self.close_th else False
            no_marker_close = True if marker_is_close else False

            #vf_flag_call->0:vfの終了、1:壁のマーカーを取りに行くvf_A、2:敵のマーカーと取りに行くvf_B、3:回避vf_C
            #color_flag->0:赤い風船、1:壁のマーカー、2:敵のマーカー、3:敵のAR、4:壁のAR
            
            if ene_is_near and ene_to_face and find_enemy: 
                #回避
                if not pre_state =="escape":
                    self.stop_sending()
                    print ""
                    print "vf c starts"
                    print ""
                    self.vf_flag_call(Int8(data=3))
                    pre_state = "escape"

            elif ((ene_is_near and ene_not_face) or ene_is_far) and in_time and no_marker_close:
                #最も点数の高い的を取りに行く
                if not (pre_state =="get_highest_marker" and pre_state == "vf_A" and pre_state == "vf_B"):
                    self.stop_sending()
                    target, _ = self.top_priority_target()
                    self.send_goal(target)
                    time.sleep(0.5)
                    pre_state = "get_highest_marker"
                
                if self.goal_reached:
                    self.target_states[target]["priority"] = -99
                    self.goal_reached = False
                    pre_state = ""
                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)

            else:
                #最も近い的を取りに行く
                if not (pre_state =="get_nearest_marker" and pre_state == "vf_A" and pre_state == "vf_B"):
                    self.stop_sending()
                    target, _ = self.nearest_target()
                    self.send_goal(target)
                    time.sleep(0.5)
                    pre_state = "get_nearest_marker"
                
                if self.goal_reached:
                    self.target_states[target]["priority"] = -99
                    self.goal_reached = False
                    pre_state = ""
                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)
        
        print "【state】:",pre_state

if __name__ == '__main__':
    rospy.init_node('send_priority_goal')
    send = SendPriorityGoal()
    send.main()