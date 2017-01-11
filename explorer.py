#!/usr/bin/env python
# coding:utf-8

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from matplotlib import pyplot as plt
import numpy as np
from math import *
import threading
from tf import transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from stateMachine import StateMachine
from costMap import costMap
from scanListener1 import scanListener
from mapDrawer import mapDrawer
import ConfigParser

class explorer(threading.Thread):
    def __init__(self, costMap, scan, drawer):
        threading.Thread.__init__(self)
        rospy.on_shutdown(self.shutdown)
        inifile = ConfigParser.SafeConfigParser()
        inifile.read('./config.ini')
        self.prePos = (0,0,0)
        self.map = costMap
        self.scan = scan
        self.drawer = drawer
        self.safe_obstacle_distance_go = inifile.getfloat('path_planner', 'safe_obstacle_distance_go')
        self.safe_obstacle_distance_turn = inifile.getfloat('path_planner', 'safe_obstacle_distance_turn')

        rospy.loginfo("Waiting for ORB_SLAM2 ...")
        while self.scan.pos_init == 0:
            pass
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        self.m = StateMachine()
        self.m.add_state("GO", self.go)
        self.m.add_state("NEXT", self.next) 
        self.m.add_state("END", self.end, end_state=1)
        self.m.setDaemon(True)
        self.m.set_start("NEXT") 
        #self.scan.listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(4.0))

    def goto(self, pos):
        quat = transformations.quaternion_from_euler(0, 0, pos[2])
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        location = Pose(Point(pos[0], pos[1], 0.000), Quaternion(x,y,z,w))
        goal = MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(goal)

    def end(self):
        rospy.loginfo("Cleaning Finished!")
        self.move_base.cancel_goal() 
            
    def go(self):
        self.goto(self.next_pos)
        preState = 0
        i = 0
        while True:
            (trans,rot) = self.scan.listener.lookupTransform('visual_odom', 'base_footprint', rospy.Time())
            fx = trans[0]
            fy = trans[1]
            state = self.move_base.get_state()
            if preState != state:
                preState = state
                print str(self.goal_states[state])
            if str(self.goal_states[state]) == 'SUCCEEDED':
                nextState = 'NEXT'        
                break
            elif i >= 2 and self.map.check_nearby_obs(fx, fy) == True:
                print 'Obstacle detected!'
                self.move_base.cancel_goal() 
                nextState = 'NEXT'        
                break
            rospy.sleep(1)
            i = i+1
        return 'NEXT'

    def next(self):
        if len(self.scan.frontier )!= 0:
            p = self.scan.frontier[0]
            p = self.map.toWorldXY(p[0], p[1])
            pos = (p[0], p[1], 0)
            self.next_pos = pos
            return 'GO'
        else:
            return 'END'
                    
    def shutdown(self):
        rospy.loginfo("Stopping the robot (pathPlaner)...")
        self.move_base.cancel_goal()
        rospy.sleep(1)

    def run(self):
        self.m.setDaemon(True)
        self.m.start()
        try:
            while True:
                self.drawer.q.put((self.map.map_data,self.scan.path))
                rospy.sleep(0.5)
        except KeyboardInterrupt:
            print 'interrupted!'
        
            
        
if __name__ == '__main__':
    try:
        rospy.init_node('plan_planner', anonymous=False)
        cmap = costMap(row = 200, col = 200, c_row = 40, c_col = 40, resolution = 0.05)
        scan = scanListener(cmap)
        scan.setDaemon(True)
        scan.start()
        drawer = mapDrawer()
        planer = pathPlaner(cmap, scan, drawer)
        planer.setDaemon(True)
        planer.start()
        drawer.run()
    except:
        print 'Main except!'
            
            
