#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import numpy as np
import tf
import sys
import os
import actionlib
import copy
from transitions.extensions import GraphMachine

#Import ROS messange type
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose
from std_msgs.msg import String, Int16MultiArray, Int8

#
import def_state
from server_receiver import ServerReceiver

#for debug
from IPython.terminal.debugger import set_trace
        
class SendPriorityGoal(ServerReceiver): #ServerReceiverの継承
    def __init__(self):
        super(SendPriorityGoal,self).__init__()
        #State Machine
        self.state = def_state.State()
        self.machine = GraphMachine(model=self.state, states=self.state.states, initial='search_enemy_distance', 
                            transitions=self.state.transitions,
                            auto_transitions=False, ordered_transitions=False,
                            title="", show_auto_transitions=False, show_conditions=False)

        #Get prameter
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.50)
        self.time_th = rospy.get_param("~time_th", 0)
        self.control_cycle = rospy.get_param("~control_cycle", 5.0)
        self.diff_theta_th = rospy.get_param("~diff_theta_th",0.7854) #pi/4
        self.near_dist_th = rospy.get_param("~near_dist_th",0.8)
        self.use_global_planner = rospy.get_param("~use_global_planner",False)
        
        #Publisher
        self.desired_goal_pub = rospy.Publisher("desired_pose", PoseStamped, queue_size=1)
        self.cancel_goal_pub = rospy.Publisher("reset_pathplan", String, queue_size=1)
        self.vf_flag_pub = rospy.Publisher("vf_flag", Int8, queue_size=1)

    def send_desired_goal(self,target_name):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.target_states[target_name]["pose"][0]
        goal.pose.position.y = self.target_states[target_name]["pose"][1]

        q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target_name]["pose"][2])
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        init_t = rospy.Time.now().to_sec()
        now_t = init_t
        if self.color_flag[1] and self.pose_target_distance(target_name,self.my_pose) <1.0* self.focus_dist:
            #vf Aスタート
            print ""
            print "vf A starts"
            print ""
            self.vf_flag_pub.publish(data=1)
            while (now_t - init_t)< self.control_cycle:
                if self.color_flag[4]:
                    break
                now_t = rospy.Time.now().to_sec()
            self.vf_flag_pub.publish(data=0)
        #if self.color_flag[2] and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R" \
        #    or target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
        elif self.color_flag[2] and self.pose_pose_distance(self.my_pose,self.enemy_pose) < self.focus_dist*5:
            #vf Bスタート
            print ""
            print "vf B starts"
            print ""
            self.vf_flag_pub.publish(Int8(data=2))
            while (now_t - init_t) < self.control_cycle:
                if self.color_flag[3]:
                    break
                now_t = rospy.Time.now().to_sec()
            self.vf_flag_pub.publish(Int8(data=0))
        else:
            print ""
            print "move base"
            print ""
            #ゴールをGlobal Plannerに送る
            self.desired_goal_pub.publish(goal)
            while (now_t - init_t) < self.control_cycle:
                now_t = rospy.Time.now().to_sec()
            self.cancel_goal_pub.publish("Stop")

    def main(self):
        while not rospy.is_shutdown():
            self.target_priority_update()
            print "【color_flag】",self.color_flag
            if self.color_flag[2]==0:
                self.ignore_enemy = True
            else:
                self.ignore_enemy = False

            if self.ignore_enemy:
                self.machine.set_state('go_to_target')
                target = self.top_priority_target()

            if self.state.state == 'search_enemy_distance':
                enemy_distance = self.pose_pose_distance(self.my_pose,self.enemy_pose)
                if enemy_distance > self.enemy_distance_th:
                    self.state.trigger('enemy_far')
                else:
                    self.state.trigger('enemy_near')

            elif self.state.state == 'get_time_left':
                if self.passed_time > self.time_th:
                    self.state.trigger('time_over')
                else:
                    self.state.trigger('in_time')

            elif self.state.state == 'get_enemy_pose':
                if self.color_flag[2] and (self.diff_theta < self.diff_theta_th):
                    self.state.trigger('can_see_and_face')
                else:
                    self.state.trigger('cannot_see_or_face')

            elif self.state.state == 'escape':
                print ""
                print "vf C starts"
                print ""
                init_t = rospy.Time.now().to_sec()
                now_t = init_t
                while (now_t - init_t) < self.control_cycle:
                    #TODO 回避動作の定義
                    self.vf_flag_pub.publish(Int8(data=3)) #vf C start
                    now_t = rospy.Time.now().to_sec()
                self.vf_flag_pub.publish(Int8(data = 0)) #vf stop
                self.state.trigger('cycle')

            elif self.state.state == 'search_near_target':
                nearest_target = self.nearest_target()
                nearest_dist = self.target_states[nearest_target]["distance"]
                if nearest_dist < self.near_dist_th:
                    self.state.trigger('near_target_exists')
                else:
                    self.state.trigger('near_target_absent')

            elif self.state.state == 'get_nearest_target':
                target = self.nearest_target()
                self.state.trigger('send_target')

            elif self.state.state == 'get_highest_target':
                target = self.highest_target()
                self.state.trigger('send_target')

            elif self.state.state == 'go_to_target':
                print "target_goal:",target
                if self.use_global_planner:
                    self.send_desired_goal(target)
                else:
                    self.send_target_goal(target)
                self.state.trigger('cycle')

            print "self.state.state:",self.state.state
            #self.show_pose()
        return

if __name__ == '__main__':
    rospy.init_node('send_priority_goal')
    send = SendPriorityGoal()
    send.main()