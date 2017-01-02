#!/usr/bin/env python
# coding:utf-8
import tf
import threading
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from collections import Counter
from math import *


class scanListener(threading.Thread):
    def __init__(self, costMap):  
        threading.Thread.__init__(self)
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber("scan", LaserScan, self.callback, queue_size = 1)
        self.listener = tf.TransformListener()
        rospy.on_shutdown(self.shutdown)
        self.map = costMap
        self.curPos = (0.0, 0.0, 0.0)
        self.front = 0

    def run(self):
        rospy.spin()

    def callback(self, data):
        d = np.array(data.ranges)
        self.front = np.mean(d[315:325]) < 0.8
        try:
            scantime = rospy.Time.now()
            (trans,rot) = self.listener.lookupTransform('odom', 'base_footprint', scantime)
            self.curPos = (trans[0],trans[1], rot[2])
            Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
        except:
            return 
        self.map.setVisit(trans[0],trans[1])
        tmp = []
        for i in range(len(data.ranges)):
            rangex = data.ranges[i]
            if rangex < 0.8:
                anglex  = data.angle_min +(i * data.angle_increment)
                x = rangex * cos(anglex)
                y = rangex * sin(anglex)
                Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
                xyz = tuple(np.dot(Rt, np.array([x, y, 0, 1.0])))[:3]
                p = self.map.toMapXY(xyz[0], xyz[1])
                tmp.append(p)

        c = Counter(tmp)
        for p in c:
            if c[p] > 5:
                self.map.setCostMap(p[0], p[1], 2)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot (scan)...")
