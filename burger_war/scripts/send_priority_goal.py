#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int8MultiArray, Int8
import json
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose
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
            {'trigger': 'cannot_see_or_face', 'source': 'get_enemy_pose', 'dest':'get_time_left'},
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

class Target(object):
    def __init__(self):
        self.name = "n"
        self.position = [0, 0, 0]
        self.time = 0
        
class SendPriorityGoal(object):
    def __init__(self):
        rospy.init_node('send_priority_goal')

        self.enemy_pose_pub = rospy.Publisher("send_enemy_pose", PoseStamped, queue_size=1)
        #State machine
        self.model = Matter()
        self.machine = GraphMachine(model=self.model, states=self.model.states, initial='search_enemy_distance', 
                            transitions=self.model.transitions,
                            auto_transitions=False, ordered_transitions=False,
                            title="", show_auto_transitions=False, show_conditions=False)

        #Get prameter
        self.side = rospy.get_param("~side", "r")
        if self.side == "r":
            self.enemy_side = "b"
        else:
            self.enemy_side = "r"
        self.focus_dist = rospy.get_param("~focous_dist",0.20) 
        self.enemy_distance_th = rospy.get_param("~enemy_distance_th",0.50)
        self.time_th = rospy.get_param("~time_th", 0)
        self.control_cycle = rospy.get_param("~control_cycle", 5.0)
        self.diff_theta_th = rospy.get_param("~diff_theta_th",0.7854) #pi/4
        self.near_dist_th = rospy.get_param("~near_dist_th",0.8)
        self.use_global_planner = rospy.get_param("~use_global_planner",False)
        self.ignore_enemy = rospy.get_param("~ignore_enemy",False)
        self.use_odom = rospy.get_param("~use_odom", True)
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

        if self.ignore_enemy:
            del self.target_states["RE_B"],self.target_states["RE_R"],self.target_states["RE_L"] ,self.target_states["BL_B"],self.target_states["BL_R"],self.target_states["BL_L"]

        #Copy previous target state
        self.target_states_pre = self.target_states
        self.last_target = Target()

        #Initialize robot position
        self.enemy_pose = PoseStamped()
        self.my_pose = PoseStamped()
        if self.side == "r":
            self.enemy_pose.pose.position.x = 1.3
            self.enemy_pose.pose.position.y = 0 
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.my_pose.pose.position.x = -1.3
            self.my_pose.pose.position.y = 0
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        elif self.side == "b":

            self.enemy_pose.pose.position.x = -1.3
            self.enemy_pose.pose.position.y = 0 
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.my_pose.pose.position.x = 1.3
            self.my_pose.pose.position.y = 0
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        self.target_pose_update()
        self.target_distance_update()

        #Initialize other variable
        self.passed_time = 0
        self.color_flag = [0,0,0,0,0,0]

        #Subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
        self.enemy_pose_sub = rospy.Subscriber('absolute_pos',PoseStamped,self.enemyposeCallback)
        self.color_flag_sub = rospy.Subscriber('color_flag_time',Int8MultiArray, self.colorCallback)
        if self.use_odom:
            self.my_pose_sub = rospy.Subscriber('odom',Odometry,self.myodomCallback)
        else:
            self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped,self.myposeCallback)
        
        #Publisher
        if self.use_global_planner:
            #Goal publisher
            self.desired_goal_pub = rospy.Publisher("desired_pose", PoseStamped, queue_size=1)
            self.cancel_goal_pub = rospy.Publisher("reset_pathplan", String, queue_size=1)
        else:
            #Action client
            self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            while not self.action.wait_for_server(rospy.Duration(5)):
                rospy.loginfo("Waiting for the move_base action server to come up")
            rospy.loginfo("The server comes up")
            # Generate Goal
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
        
        self.vf_flag_pub = rospy.Publisher("vf_flag", Int8, queue_size=1)

    def target_pose_update(self):
        #相手の（x,y,th）を持ってくる
        send_ene_pose = PoseStamped()
        send_ene_pose.header.frame_id = "map"
        send_ene_pose.pose = self.enemy_pose.pose
        self.enemy_pose_pub.publish(send_ene_pose)

        q_e = self.enemy_pose.pose.orientation
        e_e = tf.transformations.euler_from_quaternion((q_e.x,q_e.y,q_e.z,q_e.w))
        th_e = e_e[2]
        pose_e = self.enemy_pose.pose.position

        #自分の（x,y,th）を持ってくる
        q_m = self.my_pose.pose.orientation
        e_m = tf.transformations.euler_from_quaternion((q_m.x,q_m.y,q_m.z,q_m.w))
        th_m = e_m[2]
        pose_m = self.my_pose.pose.position
        self.diff_theta = th_e - th_m
        if not self.ignore_enemy:
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
                    self.target_states_pre[target_name]["player"] = self.target_states[target_name]["player"]
                    self.target_states[target_name]["player"] = info.get("player")

    def pose_target_distance(self, target_name, PoseStamped):
        diff_x = self.target_states[target_name]["pose"][0]-PoseStamped.pose.position.x
        diff_y = self.target_states[target_name]["pose"][1]-PoseStamped.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    def target_distance_update(self):
        for target_name in self.target_states:
            #自分自身の的は除外
            if self.side == 'b' and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R"):
                self.target_states[target_name]["distance"] = 99
            elif self.side == 'r' and (target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
                self.target_states[target_name]["distance"] = 99
            #他の的は距離を算出
            else:
                self.target_states[target_name]["distance"] = self.pose_target_distance(target_name, self.my_pose)
    
    def target_priority_update(self):
        for target_name in self.target_states:
            #自分自身の的は除外
            if self.side == 'b' and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R"):
                self.target_states[target_name]["priority"] = -99
            elif self.side == 'r' and (target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
                self.target_states[target_name]["priority"] = -99
            #自分自身がとった的は除外
            elif (self.side == 'b' and self.target_states[target_name]["player"] == 'b') or \
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
        self.last_enemy_target()
        self.last_target.time = server_data["time"]

    def myposeCallback(self,pose):
        self.my_pose = pose
        self.target_pose_update()
        self.target_distance_update()

    def myodomCallback(self,odom):
        self.mypose = odom.pose#.pose
        self.target_pose_update()
        self.target_distance_update()

    def enemyposeCallback(self, pose):
        self.enemy_pose = pose
        """
        if ((self.color_flag[0] + self.color_flag[2] + self.color_flag[3]) == 0) and (self.color_flag[5] < self.last_target.time):
            print "color_falg:",self.color_flag
            self.enemy_pose.pose.position.x = self.last_target.position[0]
            self.enemy_pose.pose.position.y = self.last_target.position[1]
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.last_target.position[2])
            self.enemy_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        """
        self.target_pose_update()
        self.target_distance_update()

    def colorCallback(self, array):
        self.color_flag = array.data
    
    def show_state(self): # for debug
        print("{}".format(json.dumps(self.target_states,indent=4)))
        
    def show_distnce(self):
        for target_name in self.target_states:
            print target_name, self.target_states[target_name]["distance"]

    def show_pose(self):
        for target_name in self.target_states:
            print target_name, self.target_states[target_name]["pose"]

    def send_target_goal(self,target_name):
        self.goal.target_pose.pose.position.x = self.target_states[target_name]["pose"][0]
        self.goal.target_pose.pose.position.y = self.target_states[target_name]["pose"][1]

        q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target_name]["pose"][2])
        self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        self.action.send_goal(self.goal)
        succeeded = self.action.wait_for_result(rospy.Duration(self.control_cycle))
        if succeeded:
            self.target_states[target_name]["priority"] = -99

    def send_desired_goal(self,target_name):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.target_states[target_name]["pose"][0]
        goal.pose.position.y = self.target_states[target_name]["pose"][1]
        print self.target_states[target_name]["pose"][0],self.target_states[target_name]["pose"][1]


        q = tf.transformations.quaternion_from_euler(0,0,self.target_states[target_name]["pose"][2])
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        init_t = rospy.Time.now().to_sec()
        now_t = init_t
        """
        if self.color_flag[1]:
            #vf Aスタート
            print "vf A starts"
            self.vf_flag_pub.publish(data=1)
            while (now_t - init_t)< self.control_cycle:
                if self.color_flag[4]:
                    break
                now_t = rospy.Time.now().to_sec()
            self.vf_flag_pub.publish(data=0)
        """
        if self.color_flag[2] and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R" \
            or target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
            #vf Bスタート
            print "vf B starts"
            self.vf_flag_pub.publish(data=2)
            while (now_t - init_t) < self.control_cycle:
                if self.color_flag[3]:
                    break
                now_t = rospy.Time.now().to_sec()
            self.vf_flag_pub.publish(data=0)
        else:
            #ゴールをGlobal Plannerに送る
            self.desired_goal_pub.publish(goal)
            while (now_t - init_t) < self.control_cycle:
                now_t = rospy.Time.now().to_sec()
            self.cancel_goal_pub.publish("Stop")

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

    # 相手が最後にとった的を保存
    def last_enemy_target(self):
        for target_name in self.target_states:
            if self.target_states[target_name]["player"] != self.target_states_pre[target_name]["player"]:
                if self.target_states[target_name]["player"] == self.enemy_side:
                    self.last_target.name = target_name
                    self.last_target.position = self.target_states[target_name]["pose"]

    def main(self):
        while not rospy.is_shutdown():
            self.target_priority_update()
            if self.ignore_enemy:
                self.machine.set_state('go_to_target')
                target = self.top_priority_target()

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
                print "self.color_flag:",self.color_flag
                if self.color_flag[2] and self.diff_theta < self.diff_theta_th:
                    self.model.trigger('can_see_and_face')
                else:
                    self.model.trigger('cannot_see_or_face')

            elif self.model.state == 'escape':
                init_t = rospy.Time.now().to_sec()
                now_t = init_t
                while (now_t - init_t) < self.control_cycle:
                    #TODO 回避動作の定義
                    self.vf_flag_pub.publish(date=3) #vf C start
                    now_t = rospy.Time.now().to_sec()
                self.vf_flag_pub.publish(data = 0) #vf stop
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
                print "target_goal:",target
                if self.use_global_planner:
                    self.send_desired_goal(target)
                else:
                    self.send_target_goal(target)
                self.model.trigger('cycle')

            print "self.model.state:",self.model.state
            print "self.last_enemy_pose:", self.last_target.position
            #self.show_pose()
        return

if __name__ == '__main__':
    send = SendPriorityGoal()
    send.main()