#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import tf
import sys
from nav_msgs.msg import Odometry
import os

class ServerReceiver(object):
    def __init__(self):
        #Get prameter
        self.side = rospy.get_param("~side", "r")
        self.focus_dist = rospy.get_param("~focous_dist",0.20)
        current_dir = rospy.get_param("~current_dir")
        #Subscriber
        self.server_sub = rospy.Subscriber('war_state', String, self.serverCallback)
        self.enemy_pose_sub = rospy.Subscriber('enemy_pose',PoseStamped,self.enemyposeCallback)
        self.my_pose_sub = rospy.Subscriber('my_pose',PoseStamped,self.myposeCallback)
        #Publisher

        #Initialise
        with open(current_dir+'/marker.json') as f:
            self.target_states = json.load(f)
        self.target_states["Tomato_N"]["pose"] = [-0.53,0.605+self.focus_dist,-np.pi/2]
        self.target_states["Tomato_S"]["pose"] = [-0.53,0.455-self.focus_dist,np.pi/2]
        self.target_states["Omelette_N"]["pose"] = [0.53,0.605+self.focus_dist,-np.pi/2]
        self.target_states["Omelette_S"]["pose"] = [0.53,0.455-self.focus_dist,np.pi/2]
        self.target_states["Pudding_N"]["pose"] = [-0.53,-0.455+self.focus_dist,-np.pi/2]
        self.target_states["Pudding_S"]["pose"] = [-0.53,-0.605-self.focus_dist,np.pi/2]
        self.target_states["OctopusWiener_N"]["pose"] = [0.53, -0.455+self.focus_dist,-np.pi/2]
        self.target_states["OctopusWiener_S"]["pose"] = [0.53,-0.605-self.focus_dist,np.pi/2]
        self.target_states["FriedShrimp_N"]["pose"] = [0,0.175+self.focus_dist,-np.pi/2]
        self.target_states["FriedShrimp_S"]["pose"] = [0,-0.175-self.focus_dist,np.pi/2]
        self.target_states["FriedShrimp_W"]["pose"] = [-0.175-self.focus_dist,0,0]
        self.target_states["FriedShrimp_E"]["pose"] = [0.175+self.focus_dist,0,np.pi]

        self.enemy_pose = PoseStamped()
        self.my_pose = PoseStamped()
        if self.side == "r":
            self.enemy_pose.pose.position.x = 0
            self.enemy_pose.pose.position.y = 1.3
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,-np.pi/2)
            self.enemy_pose.pose.orientation = q
            self.my_pose.pose.position.x = 0
            self.my_pose.pose.position.y = -1.3
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi/2)
            self.my_pose.pose.orientation = q
        elif self.side == "b":
            self.enemy_pose.pose.position.x = 0
            self.enemy_pose.pose.position.y = -1.3
            self.enemy_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,np.pi/2)
            self.enemy_pose.pose.orientation = q
            self.my_pose.pose.position.x = 0
            self.my_pose.pose.position.y = 1.3
            self.my_pose.pose.position.z = 0 
            q = tf.transformations.quaternion_from_euler(0,0,-np.pi/2)
            self.my_pose.pose.orientation = q
        else:
            print "Wrong side paramaeter is set"
            sys.exit()
        self.target_pose_update()
        self.target_distance_update()
        self.show_state()

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
    
    #TODO
    def target_priority_update(self):
        for target_name in self.target_states:
            if self.side == 'b' and (target_name=="BL_B" or target_name=="BL_L" or target_name=="BL_R"):
                self.target_states[target_name]["priority"] = 0
            elif self.side == 'r' and (target_name=="RE_B" or target_name=="RE_L" or target_name=="RE_R"):
                self.target_states[target_name]["priority"] = 0
            elif (self.side == 'b' and self.target_states[target_name]["player"] == 'b') or \
                (self.side == 'r' and self.target_states[target_name]["player"]=='r'):
                self.target_states[target_name]["priority"] = 0
            else:
                if (self.side == 'b' and self.target_states[target_name]["player"] == 'r') or \
                    (self.side == 'r' and self.target_states[target_name]["player"]=='b'):
                    point = float(2* self.target_states[target_name]["point"])
                else:
                    point = float(self.target_states[target_name]["point"])
                dist = float(self.target_states[target_name]["distance"])
                self.target_states[target_name]["priority"] = point - 0.5*dist

    def serverCallback(self, data):
        server_data = json.loads(data.data)
        target_info = server_data["targets"]
        self.target_player_update(target_info)

    def myposeCallback(self,pose):
        self.my_pose = pose.pose
        self.target_pose_update()
        self.target_distance_update()

    def enemyposeCallback(self, pose):
        self.enemy_pose = pose.pose
        self.target_pose_update()
        self.target_distance_update()
    
    def show_state(self): # for debug
        print("{}".format(json.dumps(self.target_states,indent=4)))

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.target_priority_update()
            self.show_state()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('server_receiver')
    receiver = ServerReceiver()
    receiver.main()