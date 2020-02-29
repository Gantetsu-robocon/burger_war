#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import math
from heapq import heappush, heappop
import json

class Node:
    def __init__(self, num, parent, cost, heuristic):
        self.num = num
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

class PriorityQueue:
    def __init__(self):
        self._container = []

    @property
    def empty(self):
        return not self._container

    def push(self, item):
        heappush(self._container, item)

    def pop(self):
        return heappop(self._container)

class Graph:
    def __init__(self, node_s, node_t, pos, weight=None):
        self.node_s = node_s
        self.node_t = node_t
        self.pos = pos
        self.num_node = len(self.pos)
        self.num_edge = len(self.node_s)
        if weight is None:
            self.weight = [1]*self.num_edge
        else:
            self.weight = weight

    def search(self, num_start, num_goal):
        self.start = num_start
        self.goal = num_goal

        frontier = PriorityQueue()
        frontier.push(Node(self.start, None, 0.0, self.heuristic(self.start)))
        self.explored = {self.start: 0.0}
        self.parent = {self.start: None}

        while not frontier.empty:
            current_node = frontier.pop()
            current_num = current_node.num

            if current_num == self.goal:
                path = [current_num]
                while self.parent[current_num] is not None:
                    current_num = self.parent[current_num]
                    path.append(current_num)
                path.reverse()
                return path

            for child in self.successors(current_num):
                for index, ns in enumerate(self.node_s):
                    if ns==current_num and self.node_t[index]==child:
                        break
                new_cost = current_node.cost + self.weight[index]

                if child not in self.explored or self.explored[child] > new_cost:
                    self.explored[child] = new_cost
                    self.parent[child] = current_num
                    frontier.push(Node(child, current_node, new_cost, self.heuristic(child)))
        return None

    def heuristic(self, num):
        pos = self.pos[num]
        dx = self.pos[self.goal][0] - pos[0]
        dy = self.pos[self.goal][1] - pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        return distance

    def successors(self, num):
        node_list = []
        for index in range(self.num_edge):
            if self.node_s[index] == num:
                node_list.append(self.node_t[index])
            if self.node_t[index] == num:
                node_list.append(self.node_s[index])
        return node_list

class GlobalPathPlan(object):
    def __init__(self, start, goal, point=None):
        self.start = start
        self.goal = goal

        self.area_s = self.where_am_I(start)
        self.area_g = self.where_am_I(goal)

        self.node_s = [1, 1, 1, 1, 3, 3, 3, 3, 5, 5, 5, 5, 7, 7, 7, 7]
        self.node_t = [2, 4, 6, 8, 2, 4, 6, 8, 2, 4, 6, 8, 2, 4, 6, 8]
        self.pos = [[0.3, 0.3], [-0.3, 0.3], [-0.3, -0.3], [0.3, -0.3], [0.75, 0.75], [-0.75, 0.75], [-0.75, -0.75], [0.75, -0.75]]
        self.weight = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

    def where_am_I(self, pos):
        x_rot = (pos[1] + pos[0]) / math.sqrt(2)
        y_rot = (pos[1] - pos[0]) / math.sqrt(2)
        if x_rot>0:
            if y_rot>0:
                return 'C'
            else:
                return 'B'
        else:
            if y_rot>0:
                return 'D'
            else:
                return 'A'

    def connect_node(self):
        if self.area_s is 'A':
            node_t_s = [3, 4, 7, 8]
        elif self.area_s is 'B':
            node_t_s = [1, 4, 5, 8]
        elif self.area_s is 'C':
            node_t_s = [1, 2, 5, 6]
        elif self.area_s is 'D':
            node_t_s = [2, 3, 6, 7]
        self.node_s[0:0] = [0, 0, 0, 0]
        self.node_t[0:0] = node_t_s
        self.pos[0:0] = [self.start]
        self.weight[0:0] = [1, 1, 1, 1]

        if self.area_g is 'A':
            node_t_g = [3, 4, 7, 8]
        elif self.area_g is 'B':
            node_t_g = [1, 4, 5, 8]
        elif self.area_g is 'C':
            node_t_g = [1, 2, 5, 6]
        elif self.area_g is 'D':
            node_t_g = [2, 3, 6, 7]
        self.node_s.extend([9, 9, 9, 9])
        self.node_t.extend(node_t_g)
        self.pos.extend([self.goal])
        self.weight.extend([1, 1, 1, 1])

    def add_theta(self):
        theta0 = math.pi/4
        theta1 = 3*math.pi/4
        theta2 = 5*math.pi/4
        theta3 = 7*math.pi/4
        THETA_DICT = {'AB': theta0,
                      'BC': theta1,
                      'CD': theta2,
                      'DA': theta3,
                      'BA': theta2,
                      'CB': theta3,
                      'DC': theta0,
                      'AD': theta1}

        desired_path = []
        for (i, num) in enumerate(self.path):
            if num == 0:
                pass
            elif num == 9:
                desired_path.append(self.goal)
            else:
                x = self.pos[num][0]
                y = self.pos[num][1]
                x_next = (self.pos[self.path[i+1]][0] + x)/2
                y_next = (self.pos[self.path[i+1]][1] + y)/2
                x_prev = (self.pos[self.path[i-1]][0] + x)/2
                y_prev = (self.pos[self.path[i-1]][1] + y)/2

                area_next = self.where_am_I([x_next, y_next])
                area_prev = self.where_am_I([x_prev, y_prev])

                theta = THETA_DICT[area_prev + area_next]
                desired_path.append([x, y, theta])
        return desired_path

    def searchPath(self):
        if self.area_s == self.area_g:
            return [self.goal]

        self.connect_node()
        self.graph = Graph(self.node_s, self.node_t, self.pos, self.weight)
        self.path = self.graph.search(0, 9)
        print("path number : " + str(self.path))
        desired_path = self.add_theta()
        return desired_path


class main():
    def __init__(self):
        rospy.init_node('global_path_planner')
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ac.wait_for_server()
        self.desired_pose_sub = rospy.Subscriber('desired_pose', PoseStamped, self.desiredPoseCallback)
        self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amclCallback)
        self.reset_pathplan_sub = rospy.Subscriber('reset_pathplan', String, self.resetPathplanCallback)

        self.succeeded_pub = rospy.Publisher('pathplan_succeeded', String, queue_size=1)

        self.desired_pose = PoseStamped()
        self.current_pose = PoseStamped()

    def desiredPoseCallback(self, data):
        self.ac.cancel_all_goals()
        self.desired_pose = data

        start = [self.current_pose.pose.position.x,
                self.current_pose.pose.position.y]
        goal = [self.desired_pose.pose.position.x,
                self.desired_pose.pose.position.y,
                2*math.acos(self.desired_pose.pose.orientation.w)]
        if self.desired_pose.pose.orientation.z < 0:
            goal[2] = - goal[2]

        pathplanner = GlobalPathPlan(start, goal)
        path = pathplanner.searchPath()
        print("path points : " + str(path))

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        for pose in path:
            self.goal.target_pose.pose.position.x =  pose[0]
            self.goal.target_pose.pose.position.y =  pose[1]
            q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
            self.goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

            rospy.loginfo("Sending goal:" + str(pose))                                          
            self.ac.send_goal(self.goal)
            
            succeeded = self.ac.wait_for_result(rospy.Duration(20))
                
            state = self.ac.get_state()

            if succeeded:
                rospy.loginfo("Succeeded: No."+"("+str(state)+")")
                self.succeeded_pub.publish('succeeded')
            else:
                rospy.loginfo("Failed: No."+"("+str(state)+")")
                self.succeeded_pub.publish('failed')
                self.ac.cancel_all_goals()
                break
            


    def amclCallback(self, data):
        self.current_pose = data.pose
    
    def resetPathplanCallback(self, data):
        self.ac.cancel_goal()




if __name__ == '__main__':
    main()
    rospy.spin()