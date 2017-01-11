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
        self.listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(60.0))
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
            (trans,rot) = self.listener.lookupTransform('odom', 'base_footprint', rospy.Time())
            Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            self.path = self.map.toMapXY(trans[0], trans[1])

        except:
            return 
        self.map.setVisit(trans[0],trans[1])
        tmp = []
        for i in range(len(data.ranges)):
            rangex = data.ranges[i]
            anglex  = data.angle_min +(i * data.angle_increment)
            x = rangex * cos(anglex)
            y = rangex * sin(anglex)
            Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            xyz = tuple(np.dot(Rt, np.array([x, y, 0, 1.0])))[:3]
            p = self.map.toMapXY(xyz[0], xyz[1])
            get_line(self.map.map_data, self.path, p)
            if rangex != 1.0:
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
