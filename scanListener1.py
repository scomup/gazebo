#!/usr/bin/env python
# coding:utf-8
import tf
import threading
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Quaternion, Point

import numpy as np
from collections import Counter
from math import *
from Bresenham import get_line

class scanListener(threading.Thread):
    def __init__(self, costMap):  
        threading.Thread.__init__(self)
        self.pos_init = 0
        self.frontier = []
        #rospy.init_node('plan_planner', anonymous=False)

        rospy.Subscriber("scan", LaserScan, self.callback_scan, queue_size = 1)

        self.listener = tf.TransformListener()
        print 'Waiting for tf ...'
        self.listener.waitForTransform('visual_odom', 'base_footprint', rospy.Time(), rospy.Duration(60.0))
        rospy.on_shutdown(self.shutdown)
        self.map = costMap
        self.pos_init = 1
        self.path = (0,0)

    def run(self):
        rospy.spin()
    def callback_scan(self, data):
        if self.pos_init == 0:
            return 
        try:
            (trans,rot) = self.listener.lookupTransform('visual_odom', 'base_footprint', rospy.Time())
            Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            self.path = self.map.toMapXY(trans[0], trans[1])

        except:
            return 
        self.map.setVisit(trans[0],trans[1])
        tmp = []
        r = np.array(data.ranges)
        theta = np.arange(data.angle_min,data.angle_max + data.angle_increment,data.angle_increment)
        a = np.c_[np.cos(theta) * r, np.sin(theta)* r, np.zeros(len(r)), np.ones(len(r))]
        points = np.dot(Rt, a.T).T[:,:2]

        for i in range(len(data.ranges)):
            p = self.map.toMapXY(points[i,0], points[i,1])
            get_line(self.map.map_data, self.path, p)
            if data.ranges[i] != 1.0:
                self.map.setCostMap(p[0], p[1], 2)
            else:
                tmp.append(p)
        for p in self.frontier:
            if self.map.chkFrontier(p) != True:
                self.frontier.remove(p)
                self.map.map_data[p[0],p[1]] = 1
        for p in tmp:
            if self.map.chkFrontier(p) == True:
                self.frontier.append(p)
        for p in self.frontier:          
            self.map.map_data[p[0],p[1]] = 3
                    
    def shutdown(self):
        rospy.loginfo("Stopping the robot (scan)...")
