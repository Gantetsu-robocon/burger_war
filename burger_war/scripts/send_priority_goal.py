#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist
from nav_msgs.msg import Odometry
import tf
import sys
from nav_msgs.msg import Odometry
import os
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SendPriorityGoal(object):
    def __init__(self):
        rospy.init_node('send_priority_goal')
        #Get prameter
        self.side = rospy.get_param("~side", "r")
        self.focus_dist = rospy.get_param("~focous_dist",0.20)
        current_dir = rospy.get_param("~current_dir")

        #Initialize target position
        with open(current_dir+'/marker.json') as f:
            self.target_states = json.load(f)
        self.target_states["Tomato_N"]["pose"][0] += self.focus_dist
        self.target_states["Tomato_S"]["pose"][0] -= self.focus_dist
        self.target_states["Omelette_N"]["pose"][0] += self.focus_dist 
        self.target_states["Omelette_S"]["pose"][0] -= self.focus_dist
        self.target_states["Pudding_N"]["pose"][0] += self.focus_dist
        self.target_states["Pudding_S"]["pose"][0] -= self.focus_dist
        self.target_states["OctopusWiener_N"]["pose"][0] += self.focus_dist
        self.target_states["OctopusWiener_S"]["pose"][0] -= self.focus_dist
        self.target_states["FriedShrimp_N"]["pose"][0] += self.focus_dist
        self.target_states["FriedShrimp_S"]["pose"][0] -= self.focus_dist
        self.target_states["FriedShrimp_W"]["pose"][1] += self.focus_dist
        self.target_states["FriedShrimp_E"]["pose"][1] -= self.focus_dist

        #Initialize robot position
        self.enemy_pose = PoseStamped()
        self.my_pose = PoseStamped()
        if self.side == "r":
            self.enemy_pose.pose.position.x = 1.3
            self.enemy_pose.pose.position.y = 0 
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.enemy_pose.pose.orientation = q
            self.my_pose.pose.position.x = -1.3
            self.my_pose.pose.position.y = 0
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.my_pose.pose.orientation = q
        elif self.side == "b":
            self.enemy_pose.pose.position.x = -1.3
            self.enemy_pose.pose.position.y = 0 
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.enemy_pose.pose.orientation = q
            self.my_pose.pose.position.x = 1.3
            self.my_pose.pose.position.y = 0
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.my_pose.pose.orientation = q
        else:
            print "Wrong side paramaeter is set"
            sys.exit()
        self.target_pose_update()
        self.target_distance_update()

        #Subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
        self.enemy_pose_sub = rospy.Subscriber('abs_enemy_pose',PoseStamped,self.enemyposeCallback)
        #self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped,self.myposeCallback)
        self.my_pose_sub = rospy.Subscriber('odom',Odometry,self.myodomCallback)

        #Action client
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        while not self.action.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.loginfo("The server comes up")

        # Generate Goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

    def target_pose_update(self):
        q_e = self.enemy_pose.pose.orientation
        e_e = tf.transformations.euler_from_quaternion((q_e[0],q_e[1],q_e[2],q_e[3]))
        th_e = e_e[2]
        pose_e = self.enemy_pose.pose.position
        q_m = self.my_pose.pose.orientation
        e_m = tf.transformations.euler_from_quaternion((q_m[0],q_m[1],q_m[2],q_m[3]))
        th_m = e_m[2]
        pose_m = self.my_pose.pose.position

        if self.side == "r":
            self.target_states["BL_L"]["pose"] = [pose_e.x-(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y+(0.07+self.focus_dist)*np.cos(th_e), th_e-np.pi/2]
            self.target_states["BL_R"]["pose"] = [pose_e.x+(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y-(0.07+self.focus_dist)*np.cos(th_e), th_e+np.pi/2]
            self.target_states["BL_B"]["pose"] = [pose_e.x-(0.1+self.focus_dist)*np.cos(th_e),
                pose_e.y-(0.1+self.focus_dist)*np.sin(th_e), th_e]
            self.target_states["RE_L"]["pose"] = [pose_m.x-(0.07+self.focus_dist)*np.sin(th_m),
                pose_m.y+(0.07+self.focus_dist)*np.cos(th_m), th_m-np.pi/2]
            self.target_states["RE_R"]["pose"] = [pose_m.x+(0.07+self.focus_dist)*np.sin(th_m),
                pose_m.y-(0.07+self.focus_dist)*np.cos(th_m), th_m+np.pi/2]
            self.target_states["RE_B"]["pose"] = [pose_m.x-(0.1+self.focus_dist)*np.cos(th_m),
                pose_m.y-(0.1+self.focus_dist)*np.sin(th_m), th_m]
        elif self.side == "b":
            self.target_states["RE_L"]["pose"] = [pose_e.x-(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y+(0.07+self.focus_dist)*np.cos(th_e), th_e-np.pi/2]
            self.target_states["RE_R"]["pose"] = [pose_e.x+(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y-(0.07+self.focus_dist)*np.cos(th_e), th_e+np.pi/2]
            self.target_states["RE_B"]["pose"] = [pose_e.x-(0.1+self.focus_dist)*np.cos(th_e),
                pose_e.y-(0.1+self.focus_dist)*np.sin(th_e), th_e]
            self.target_states["BL_L"]["pose"] = [pose_m.x-(0.07+self.focus_dist)*np.sin(th_m),
                pose_m.y+(0.07+self.focus_dist)*np.cos(th_m), th_m-np.pi/2]
            self.target_states["BL_R"]["pose"] = [pose_m.x+(0.07+self.focus_dist)*np.sin(th_m),
                pose_m.y-(0.07+self.focus_dist)*np.cos(th_m), th_m+np.pi/2]
            self.target_states["BL_B"]["pose"] = [pose_m.x-(0.1+self.focus_dist)*np.cos(th_m),
                pose_m.y-(0.1+self.focus_dist)*np.sin(th_m), th_m]

    def target_player_update(self,target_data):
        for info in target_data:
            for target_name in self.target_states:
                if info.get("name") == target_name:
                    self.target_states[target_name]["player"] = info.get("player")

    def target_distance_update(self):
        position = self.my_pose.pose.position
        for target_name in self.target_states:
            diff_x = self.target_states[target_name]["pose"][0]-position.x
            diff_y = self.target_states[target_name]["pose"][1]-position.y
            self.target_states[target_name]["distance"] = np.sqrt(diff_x**2+diff_y**2)
    
    def target_priority_update(self):
        for target_name in self.target_states:
            #自分自身がとった的あるいは自分自身の的は除外
            if self.side == 'b' and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R"):
                self.target_states[target_name]["priority"] = -999
            elif self.side == 'r' and (target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
                self.target_states[target_name]["priority"] = -999
            elif (self.side == 'b' and self.target_states[target_name]["player"] == 'b') or \
                (self.side == 'r' and self.target_states[target_name]["player"]=='r'):
                self.target_states[target_name]["priority"] = -999
            else:
                #相手がとっている的のポイントは２倍
                if (self.side == 'b' and self.target_states[target_name]["player"] == 'r') or \
                    (self.side == 'r' and self.target_states[target_name]["player"]=='b'):
                    point = float(2* self.target_states[target_name]["point"])
                else:
                    point = float(self.target_states[target_name]["point"])
                dist = float(self.target_states[target_name]["distance"])
                #TODO 優先度の算出
                self.target_states[target_name]["priority"] = -dist #+point

    def serverCallback(self, data):
        server_data = json.loads(data.data)
        target_info = server_data["targets"]
        self.target_player_update(target_info)

    def myposeCallback(self,pose):
        self.my_pose = pose.pose
        self.target_pose_update()
        self.target_distance_update()

    def myodomCallback(self,odom):
        self.mypose = odom.pose.pose
        self.target_pose_update()
        self.target_distance_update()

    def enemyposeCallback(self, pose):
        self.enemy_pose = pose.pose
        self.target_pose_update()
        self.target_distance_update()
    
    def show_state(self): # for debug
        print("{}".format(json.dumps(self.target_states,indent=4)))

    def send_goal(self,goal_name,goal_point):
        self.goal.target_pose.pose.position.x = goal_point[0]
        self.goal.target_pose.pose.position.y = goal_point[1]

        q = tf.transformations.quaternion_from_euler(0,0,goal_point[2])
        self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        self.action.send_goal(self.goal)
        succeeded = self.action.wait_for_result(rospy.Duration(30))
        if succeeded:
            self.target_states[goal_name]["priority"] = -999

    def top_priority_target(self):
        top_pri_name = "Tomato_N"
        for target_name_i in self.target_states:
            if self.target_states[target_name_i]["priority"] > self.target_states[top_pri_name]["priority"]:
                top_pri_name = target_name_i
        return top_pri_name

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.target_priority_update()
            top_priority_name = self.top_priority_target()
            top_priority_point = self.target_states[top_priority_name]["pose"]
            #print "top_priotity_target:",self.top_priotity_target()
            self.send_goal(top_priority_name,top_priority_point)
            self.show_state()
            r.sleep()

if __name__ == '__main__':
    send = SendPriorityGoal()
    send.main()