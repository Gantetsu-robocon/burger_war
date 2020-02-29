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

from transitions.extensions import GraphMachine

class Matter(object):
    def __init__(self):
        self.states = ['search_enemy_distance', 'get_time_left', 'get_enemy_pose','get_nearest_target',
            'search_near_target','get_highest_target','go_to_target','escape']

        self.transitions = [
            {'trigger': 'enemy_far', 'source': 'search_enemy_distance', 'dest':'get_time_left'},
            {'trigger': 'enemy_near', 'source': 'search_enemy_distance', 'dest':'get_enemy_pose'},
            {'trigger': 'cannnot_see_or_face', 'source': 'get_enemy_pose', 'dest':'get_time_left'},
            {'trigger': 'can_see_and_face', 'source': 'get_enemy_pose', 'dest':'escape'},
            {'trigger': 'time_over', 'source': 'get_time_left', 'dest':'get_nearest_target'},
            {'trigger': 'in_time', 'source': 'get_time_left', 'dest':'search_near_target'},
            {'trigger': 'near_target_exists', 'source': 'search_near_target', 'dest':'get_nearest_target'},
            {'trigger': 'near_target_absent', 'source': 'search_near_target', 'dest':'get_highest_target'},
            {'trigger': 'send_target', 'source': 'get_nearest_target', 'dest':'go_to_target'},
            {'trigger': 'send_target', 'source': 'get_highest_target', 'dest':'go_to_target'},
            {'trigger': 'cycle', 'source': 'go_to_target', 'dest':'search_enemy_distance'},
            {'trigger': 'cycle', 'source': 'escape', 'dest':'search_enemy_distance'}
        ]

class SendPriorityGoal(object):
    def __init__(self):
        rospy.init_node('send_priority_goal')

        #State machine
        self.model = Matter()
        self.machine = GraphMachine(model=self.model, states=self.model.states, initial='search_enemy_distance', 
                            transitions=self.model.transitions,
                            auto_transitions=False, ordered_transitions=False,
                            title="", show_auto_transitions=False, show_conditions=False)

        #Get prameter
        self.side = rospy.get_param("~side", "r")
        self.focus_dist = rospy.get_param("~focous_dist",0.20) 
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.50)
        self.time_th = rospy.get_param("~time_th",10)
        self.control_cycle = rospy.get_param("~control_cycle", 5.0)
        self.diff_theta_th = rospy.get_param("~diff_theta_th",0.7854) #pi/4
        self.near_dist_th = rospy.get_param("~near_dist_th",0.8)
        current_dir = rospy.get_param("~current_dir","/home/koki/catkin_ws/src/burger_war/burger_war/scripts")

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

        if self.side == "r":
            del self.target_states["RE_B"],self.target_states["RE_R"],self.target_states["RE_L"]
        elif self.side == "b":
            del self.target_states["BL_B"],self.target_states["BL_R"],self.target_states["BL_L"]
        else:
            print "Wrong side paramaeter is set"
            sys.exit()


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

        self.target_pose_update()
        self.target_distance_update()

        #Initialize other variable
        self.passed_time = 0

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
        self.diff_theta = th_e - th_m

        if self.side == "r":
            self.target_states["BL_L"]["pose"] = [pose_e.x-(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y+(0.07+self.focus_dist)*np.cos(th_e), th_e-np.pi/2]
            self.target_states["BL_R"]["pose"] = [pose_e.x+(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y-(0.07+self.focus_dist)*np.cos(th_e), th_e+np.pi/2]
            self.target_states["BL_B"]["pose"] = [pose_e.x-(0.1+self.focus_dist)*np.cos(th_e),
                pose_e.y-(0.1+self.focus_dist)*np.sin(th_e), th_e]
            #self.target_states["RE_L"]["pose"] = [pose_m.x-(0.07+self.focus_dist)*np.sin(th_m),
            #    pose_m.y+(0.07+self.focus_dist)*np.cos(th_m), th_m-np.pi/2]
            #self.target_states["RE_R"]["pose"] = [pose_m.x+(0.07+self.focus_dist)*np.sin(th_m),
            #    pose_m.y-(0.07+self.focus_dist)*np.cos(th_m), th_m+np.pi/2]
            #self.target_states["RE_B"]["pose"] = [pose_m.x-(0.1+self.focus_dist)*np.cos(th_m),
            #    pose_m.y-(0.1+self.focus_dist)*np.sin(th_m), th_m]
        elif self.side == "b":
            self.target_states["RE_L"]["pose"] = [pose_e.x-(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y+(0.07+self.focus_dist)*np.cos(th_e), th_e-np.pi/2]
            self.target_states["RE_R"]["pose"] = [pose_e.x+(0.07+self.focus_dist)*np.sin(th_e),
                pose_e.y-(0.07+self.focus_dist)*np.cos(th_e), th_e+np.pi/2]
            self.target_states["RE_B"]["pose"] = [pose_e.x-(0.1+self.focus_dist)*np.cos(th_e),
                pose_e.y-(0.1+self.focus_dist)*np.sin(th_e), th_e]
            #self.target_states["BL_L"]["pose"] = [pose_m.x-(0.07+self.focus_dist)*np.sin(th_m),
            #    pose_m.y+(0.07+self.focus_dist)*np.cos(th_m), th_m-np.pi/2]
            #self.target_states["BL_R"]["pose"] = [pose_m.x+(0.07+self.focus_dist)*np.sin(th_m),
            #    pose_m.y-(0.07+self.focus_dist)*np.cos(th_m), th_m+np.pi/2]
            #self.target_states["BL_B"]["pose"] = [pose_m.x-(0.1+self.focus_dist)*np.cos(th_m),
            #    pose_m.y-(0.1+self.focus_dist)*np.sin(th_m), th_m]

    def target_player_update(self,target_data):
        for info in target_data:
            for target_name in self.target_states:
                if info.get("name") == target_name:
                    self.target_states[target_name]["player"] = info.get("player")

    def pose_target_distance(self, target_name, PoseStamped):
        diff_x = self.target_states[target_name]["pose"][0]-PoseStamped.pose.position.x
        diff_y = self.target_states[target_name]["pose"][1]-PoseStamped.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    def target_distance_update(self):
        for target_name in self.target_states:
            self.target_states[target_name]["distance"] = self.pose_target_distance(target_name, self.my_pose)
    
    def target_priority_update(self):
        for target_name in self.target_states:
            #自分自身がとった的は除外
            if (self.side == 'b' and self.target_states[target_name]["player"] == 'b') or \
                (self.side == 'r' and self.target_states[target_name]["player"]=='r'):
                self.target_states[target_name]["priority"] = -99
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
        self.passed_time = server_data["time"]

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

    def send_target_goal(self,target_name):
        self.goal.target_pose.pose.position.x = self.target_states[target_name]["pose"][0]
        self.goal.target_pose.pose.position.y = self.target_states[target_name]["pose"][1]

        q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target_name]["pose"][2])
        self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        self.action.send_goal(self.goal)
        succeeded = self.action.wait_for_result(rospy.Duration(self.control_cycle))
        if succeeded:
            self.target_states[target_name]["priority"] = -99

    def top_priority_target(self):
        top_pri_name = "Tomato_N"
        for target_name in self.target_states:
            if self.target_states[target_name]["priority"] > self.target_states[top_pri_name]["priority"]:
                top_pri_name = target_name
        return top_pri_name

    def nearest_target(self):
        target_list = [ d for d in self.target_states if not self.target_states[d].get("priority") == -99 ]
        nearest_name = target_list[0]
        for target_name in target_list:
            if self.target_states[target_name]["distance"] < self.target_states[nearest_name]["distance"]:
                nearest_name = target_name
        return nearest_name

    def highest_target(self):
        target_list = [ d for d in self.target_states if not self.target_states[d].get("priority") == -99 ]
        highest_name = target_list[0]
        for target_name in target_list:
            if self.target_states[target_name]["point"] > self.target_states[highest_name]["point"]:
                highest_name = target_name
        return highest_name

    def pose_pose_distance(self,PoseStamped_1,PoseStamped_2):
        diff_x = PoseStamped_2.pose.position.x - PoseStamped_1.pose.position.x
        diff_y = PoseStamped_2.pose.position.y - PoseStamped_1.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)


    def main(self):
        while not rospy.is_shutdown():
            self.target_priority_update()

            if self.model.state == 'search_enemy_distance':
                enemy_distance = self.pose_pose_distance(self.my_pose,self.enemy_pose)
                if enemy_distance > self.enemy_distance_th:
                    self.model.trigger('enemy_far')
                else:
                    self.model.trigger('enemy_near')

            elif self.model.state == 'get_time_left':
                if self.passed_time > self.time_th:
                    self.model.trigger('time_over')
                else:
                    self.model.trigger('in_time')

            elif self.model.state == 'get_enemy_pose':
                enemy_pose = True #TODO 敵が見えるかどうかのFlag
                if enemy_pose and self.diff_theta < self.diff_theta_th:
                    self.model.trigger('can_see_and_face')
                else:
                    self.model.trigger('cannot_see_or_face')

            elif self.model.state == 'escape':
                init_t = rospy.time.now
                now_t = init_t
                while now_t - init_t > self.control_cycle:
                    #TODO 回避動作の定義
                    now_t = rospy.time.now
                    print now_t
                self.model.trigger('cycle')

            elif self.model.state == 'search_near_target':
                nearest_target = self.nearest_target()
                nearest_dist = self.target_states[nearest_target]["distance"]
                if nearest_dist < self.near_dist_th:
                    self.model.trigger('near_target_exists')
                else:
                    self.model.trigger('near_target_absent')

            elif self.model.state == 'get_nearest_target':
                target = self.nearest_target()
                self.model.trigger('send_target')

            elif self.model.state == 'get_highest_target':
                target = self.highest_target()
                self.model.trigger('send_target')

            elif self.model.state == 'go_to_target':
                self.send_target_goal(target)
                print "target_goal:",target
                self.model.trigger('cycle')

            print "self.model.state:",self.model.state
        return

if __name__ == '__main__':
    send = SendPriorityGoal()
    send.main()