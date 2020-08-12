#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Import
import rospy
import json
import tf
import copy
import numpy as np

#Import ROS message type
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16MultiArray, Int8, Bool
from sensor_msgs.msg import LaserScan

class Target(object):
    def __init__(self,position):
        self.name = "n"
        self.position = position
        self.time = 0

class ServerReceiver(object):
    def __init__(self):
        #Get parameter
        self.side = rospy.get_param("~side", "r")
        self.enemy_side = "b" if self.side =="r" else "r"
        self.focus_dist = rospy.get_param("~focous_dist",0.20) 
        current_dir = rospy.get_param("~current_dir","/home/koki/catkin_ws/src/burger_war/burger_war/scripts")

        #Initialize wall target states
        with open(current_dir+'/marker_pose.json') as f:
            self.wall_target_states = json.load(f)
        self.wall_target_states["Tomato_N"]["pose"][0] += self.focus_dist
        self.wall_target_states["Omelette_N"]["pose"][0] += self.focus_dist 
        self.wall_target_states["Pudding_S"]["pose"][0] -= self.focus_dist
        self.wall_target_states["OctopusWiener_S"]["pose"][0] -= self.focus_dist
        self.wall_target_states["FriedShrimp_N"]["pose"][0] += self.focus_dist
        self.wall_target_states["FriedShrimp_S"]["pose"][0] -= self.focus_dist

        #Initialize enemy target states
        if self.side == "r":
            self.enemy_target_states = {"BL_B":"n","BL_R":"n","BL_L":"n"}
        else:
            self.enemy_target_states = {"RE_B":"n","RE_R":"n","RE_L":"n"}

        #Copy previous target state
        self.wall_target_states_pre = copy.deepcopy(self.wall_target_states)

        #Initialize robot position
        self.enemy_pose = PoseStamped()
        self.my_pose = PoseStamped()
        if self.side == "r":
            self.enemy_pose.pose.position = Point(1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.enemy_pose.pose.position = Point(-1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.last_target = Target([1.3,0,np.pi])

        else:
            self.enemy_pose.pose.position = Point(-1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,0)
            self.enemy_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.my_pose.pose.position = Point(1.3,0,0)
            q = tf.transformations.quaternion_from_euler(0,0,np.pi)
            self.my_pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

            self.last_target = Target([-1.3,0,0])

        #Initialize other variables
        self.passed_time = 0
        self.color_flag = [0,0,0,0,0,0]
        self.lidar_flag = False #敵がLidarで見えるかどうか
        self.succeeded_goal = False
        self.near_backwall = False
        self.enemy_lost = False
        self.target_distance_update()

        #Subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
        self.enemy_pose_sub = rospy.Subscriber('absolute_pos',PoseStamped,self.enemyposeCallback)
        self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped,self.myposeCallback)
        self.color_flag_sub = rospy.Subscriber('color_flag_time',Int16MultiArray, self.colorCallback)
        self.lidar_flag_sub = rospy.Subscriber('lidar_flag',Bool, self.lidarCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarDataCallback)

    #Update target information
    def target_player_update(self,target_data):
        for info in target_data:
            target_name = info.get("name")
            if target_name in self.wall_target_states:
                self.wall_target_states_pre[target_name]["player"] = self.wall_target_states[target_name]["player"]
                self.wall_target_states[target_name]["player"] = info.get("player")
            elif target_name in self.enemy_target_states:
                self.enemy_target_states[target_name] = info.get("player")

    def target_distance_update(self):
        for target_name in self.wall_target_states:
            self.wall_target_states[target_name]["distance"] = self.target_distance(target_name)
    
    # 相手が最後にとった的を保存
    def last_enemy_target(self):
        for target_name in self.wall_target_states:
            if self.wall_target_states[target_name]["player"] != self.wall_target_states_pre[target_name]["player"]:
                if self.wall_target_states[target_name]["player"] == self.enemy_side:
                    self.last_target.name = target_name
                    self.last_target.position = self.wall_target_states[target_name]["pose"]

    #Callback method
    def serverCallback(self, data):
        server_data = json.loads(data.data)
        target_info = server_data["targets"]
        self.target_player_update(target_info)
        self.passed_time = server_data["time"]
        self.last_enemy_target()
        self.last_target.time = server_data["time"]

    def myposeCallback(self,pose):
        self.my_pose = pose
        self.target_distance_update()

    def enemyposeCallback(self, pose):
        self.enemy_pose = pose
        if not ((self.color_flag[0] + self.color_flag[2] + self.color_flag[3]) == 0 and (self.lidar_flag==False)):
            #敵が見える
            self.enemy_catch_time = rospy.Time.now().to_sec()
            self.enemy_lost = False
        else:
            #敵が一切見えない
            diff_time = rospy.Time.now().to_sec()- self.enemy_catch_time
            if diff_time > 10.0:
                #かつ一定時間以上敵が見えない
                self.enemy_lost = True

    def colorCallback(self, array):
        self.color_flag = array.data
    
    def lidarCallback(self, data):
        self.lidar_flag = data.data

    def lidarDataCallback(self,data):
        backward_scan = data.ranges[145:215]
        backward_scan = [x for x in backward_scan if x > 0.1]
        forward_scan = data.ranges[:20]+data.ranges[-20:]
        forward_scan = [x for x in backward_scan if x > 0.1]
        #後ろの壁との距離が0.28未満のとき後退を止める
        if min(backward_scan) < 0.28:
            self.near_backwall = True
        else:
            self.near_backwall = False
        #前の壁との距離が0.20未満のとき前進を止める
        if min(forward_scan) < 0.20:
            self.near_frontwall = True
        else:
            self.near_frontwall = False

    #Choose target
    def nearest_target(self):
        target_list = [ d for d in self.wall_target_states if not self.wall_target_states[d].get("player") == self.side]
        nearest_name = target_list[0]
        for target_name in target_list:
            if self.wall_target_states[target_name]["distance"] < self.wall_target_states[nearest_name]["distance"]:
                nearest_name = target_name
        return nearest_name

    def highest_target(self):
        target_list = [ d for d in self.wall_target_states if self.wall_target_states[d].get("player") == self.enemy_side]
        if not target_list:
            return self.nearest_target()
        else:
            highest_name = target_list[0]
            for target_name in target_list:
                if self.wall_target_states[target_name]["distance"] > self.wall_target_states[highest_name]["distance"]:
                    highest_name = target_name
        return highest_name

    #Calculate distance
    def target_distance(self, target_name):
        diff_x = self.wall_target_states[target_name]["pose"][0]-self.my_pose.pose.position.x
        diff_y = self.wall_target_states[target_name]["pose"][1]-self.my_pose.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    def enemy_distance(self):
        diff_x = self.enemy_pose.pose.position.x - self.my_pose.pose.position.x
        diff_y = self.enemy_pose.pose.position.y - self.my_pose.pose.position.y
        return np.sqrt(diff_x**2+diff_y**2)

    #Debug method
    def show_wall_target_states(self):
        print("{}".format(json.dumps(self.wall_target_states,indent=4)))
        
    def show_distnce(self):
        for target_name in self.wall_target_states:
            print target_name, self.wall_target_states[target_name]["distance"]

    def show_pose(self):
        for target_name in self.wall_target_states:
            print target_name, self.wall_target_states[target_name]["pose"]