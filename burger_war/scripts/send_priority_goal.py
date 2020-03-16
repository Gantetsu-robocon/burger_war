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
import math

#Import ROS topic type
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose
from std_msgs.msg import String, Int16MultiArray, Int8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Import ROS service type
from burger_war.srv import DesiredPose, VisualFeedbackFlag
from std_srvs.srv import Empty,EmptyResponse

#分割
import def_state
from server_receiver import ServerReceiver

#for debug
#from IPython.terminal.debugger import set_trace
        
class SendPriorityGoal(ServerReceiver): #ServerReceiverの継承
    def __init__(self):
        super(SendPriorityGoal,self).__init__()
        #Get parameter
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.50)
        self.enemy_close = rospy.get_param("~enemy_close", 0.2)
        self.escape_distance = rospy.get_param("~escape_distance", 0.6)
        self.time_th = rospy.get_param("~time_th", 150)
        self.close_th = rospy.get_param("~close_th",0.8)
        self.vf_B_dist = rospy.get_param("~vf_B_dist",0.5)
        self.update_enemy_time = rospy.get_param("~update_enemy_time", 2.0)
        self.stack_time = rospy.get_param("~stack",5.0)
        
        #Service Server
        self.path_success_srv = rospy.Service("pathplan_succeeded",Empty, self.successCallback)

        #Service Client
        rospy.wait_for_service("desired_pose")
        self.desired_pose_call = rospy.ServiceProxy("desired_pose",DesiredPose)
        rospy.wait_for_service("reset_pathplan")
        self.cancel_goal_call = rospy.ServiceProxy("reset_pathplan", Empty)
        rospy.wait_for_service("vf_flag")
        self.vf_flag_call = rospy.ServiceProxy("vf_flag", VisualFeedbackFlag) 

        #Action Client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()

        #Publisher for rviz
        self.goal_pub = rospy.Publisher("desired_goal",PoseStamped, queue_size=1)

        #Velocity dealer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.vel_sub = rospy.Subscriber('cmd_vel',  Twist, self.velCallback)
        self.pre_vel_time = 12

        self.goal_reached = False
        
    def velCallback(self, data):
        if not (data.linear.x == 0.0 and data.angular.z == 0.0):
            self.pre_vel_time = rospy.Time.now().to_sec()

    def successCallback(self,req):
        self.goal_reached = True
        return EmptyResponse()

    def stop_sending(self):
        self.cancel_goal_call()
        self.vf_flag_call(Int8(data=0))

    def send_goal(self,target):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        if target in self.enemy_target:
            th = math.atan2(self.enemy_pose.pose.position.y-self.my_pose.pose.position.y,
                            self.enemy_pose.pose.position.x-self.my_pose.pose.position.x)
            x = self.enemy_pose.pose.position.x - 0.1*math.cos(th)
            y = self.enemy_pose.pose.position.y - 0.1*math.sin(th)
            goal.pose.position.x = x
            goal.pose.position.y = y
            q = tf.transformations.quaternion_from_euler(0,0,th)
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        elif type(target) is unicode:
            goal.pose.position.x = self.target_states[target]["pose"][0]
            goal.pose.position.y = self.target_states[target]["pose"][1]
            q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target]["pose"][2])
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        elif type(target) is list:
            goal.pose.position.x = target[0]
            goal.pose.position.y = target[1]
            q = tf.transformations.quaternion_from_euler(0,0,target[2])
            goal.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            
        self.goal_pub.publish(goal) #for rviz
        self.desired_pose_call(goal)

    def vf(self,previous_state,target):
        pre_state = previous_state
        if self.color_flag[2] and self.enemy_distance() < self.vf_B_dist:
            if not pre_state == "vf":
                self.stop_sending()
                self.vf_flag_call(Int8(data=2))
                pre_state = "vf"
                print ""
                print "【state】:",pre_state
                print ""

            if self.color_flag[3]: #AR読み取れたら
                print "AR読み取り成功"
                self.vf_flag_call(Int8(data=0))
                if type(target) is unicode:
                    self.target_states[target]["priority"] = -99
                pre_state = ""

        if pre_state =="vf":
            if (not self.color_flag[2]) or self.enemy_distance < self.enemy_close or \
                 self.near_frontwall: #マーカーが見えなくなるor敵と近づきすぎるor壁と近づきすぎる
                print "AR読み取り失敗"
                self.vf_flag_call(Int8(data=0))
                pre_state = ""

        return pre_state

    def main(self):
        pre_state = ""
        target = ""
        send_time = 0
        r = rospy.Rate(1)
        default_vf_B_dist = self.vf_B_dist

        while not rospy.is_shutdown():
            self.target_priority_update()

            #敵との距離が近いか
            ene_is_near = True if self.enemy_distance() < self.enemy_distance_th else False
            #敵との距離が近すぎるか
            ene_is_close = True if self.enemy_distance() < self.enemy_close else False
            #相手が見えているかどうか
            find_enemy = True if self.color_flag[2] else False
            #時間が残っているか
            in_time = True if self.passed_time < self.time_th else False
            #かなり近いところに壁のマーカーがあるか
            _, nearest_dist = self.nearest_target()
            marker_is_close = True if nearest_dist < self.close_th else False
            no_marker_close = True if not marker_is_close else False
            #敵の位置を更新するか
            if target in self.enemy_target and rospy.Time.now().to_sec() - send_time > self.update_enemy_time:
                update_enemy = True
            else:
                update_enemy = False

            #vf_flag_call->0:vfの終了、1:壁のマーカーを取りに行くvf_A、2:敵のマーカーと取りに行くvf_B、3:回避vf_C
            #color_flag->0:赤い風船、1:壁のマーカー、2:敵のマーカー、3:敵のAR、4:壁のAR

            #敵の的を2個以上取っていない状態ならすぐに敵の方へvf
            count = 0
            for enemy_target in self.enemy_target:
                if self.target_states[enemy_target]["player"] == self.side :
                    count += 1
            if count <2:
                self.vf_B_dist = 1.2
            else:
                self.vf_B_dist = default_vf_B_dist

            vel_time_diff = rospy.Time.now().to_sec() - self.pre_vel_time
            
            if (ene_is_close and find_enemy) or vel_time_diff > self.stack_time: 
                #回避
                if not pre_state =="escape":
                    self.stop_sending()
                    self.vf_flag_call(Int8(data=3))
                    pre_state = "escape"
                    print ""
                    print "【state】:",pre_state
                    print ""
                    while not rospy.is_shutdown():
                        if self.enemy_distance() > self.enemy_distance_th or self.near_backwall:
                            self.vf_flag_call(Int8(data=0))
                            pre_state = ""
                            break
                        r.sleep()
            
            elif ene_is_near and self.lidar_flag:
                #相手の方を向く
                if not pre_state =="vf":
                    if (not pre_state == "turn_to_enemy" ) or update_enemy:
                        self.stop_sending()
                        th = math.atan2(self.enemy_pose.pose.position.y-self.my_pose.pose.position.y,
                                        self.enemy_pose.pose.position.x-self.my_pose.pose.position.x)
                        x = self.my_pose.pose.position.x
                        y = self.my_pose.pose.position.y
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = x
                        goal.target_pose.pose.position.y = y
                        q = tf.transformations.quaternion_from_euler(0,0,th)
                        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
                        self.ac.send_goal(goal)
                        send_time = rospy.Time.now().to_sec()
                        pre_state = "turn_to_enemy"
                        print ""
                        print "【state】:",pre_state
                        print ""

                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)

            elif in_time and no_marker_close:
                #最も点数の高い的を取りに行く
                if not pre_state =="vf":
                    if (not pre_state == "get_highest_marker" ) or update_enemy:
                        self.stop_sending()
                        target, point = self.top_priority_target()
                        self.send_goal(target)
                        send_time = rospy.Time.now().to_sec()
                        time.sleep(0.5)
                        pre_state = "get_highest_marker"
                        print ""
                        print "【state】:",pre_state
                        print "  target:", target
                        print ""

                if self.goal_reached or self.target_states[target]["player"]==self.side:
                    self.stop_sending()
                    print "ゴール到達"
                    if type(target) is unicode:
                        self.target_states[target]["priority"] = -99
                    self.goal_reached = False
                    pre_state = ""
                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)

            else:
                #最も近い的を取りに行く
                if not pre_state =="vf":
                    if (not pre_state == "get_nearest_marker" ) or update_enemy:
                        self.stop_sending()
                        target, _ = self.nearest_target()
                        self.send_goal(target)
                        send_time = rospy.Time.now().to_sec()
                        time.sleep(0.5)
                        pre_state = "get_nearest_marker"
                        print ""
                        print "【state】:",pre_state
                        print "  target:", target
                        print ""
                
                if self.goal_reached or self.target_states[target]["player"]==self.side:
                    self.stop_sending()
                    print "ゴール到達"
                    if type(target) is unicode:
                        self.target_states[target]["priority"] = -99
                    self.goal_reached = False
                    pre_state = ""

                #条件がそろえばvf
                pre_state = self.vf(pre_state,target)
                r.sleep()
        

if __name__ == '__main__':
    rospy.init_node('send_priority_goal')
    send = SendPriorityGoal()
    send.main()