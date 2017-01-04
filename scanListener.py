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


class scanListener(threading.Thread):
    def __init__(self, costMap):  
        threading.Thread.__init__(self)
        self.pos_init = 0
        rospy.init_node('listener', anonymous=False)

        rospy.Subscriber("scan", LaserScan, self.callback_scan, queue_size = 1)

        self.listener = tf.TransformListener()
        print 'Waiting for tf ...'
        self.listener.waitForTransform('visual_odom', 'base_footprint', rospy.Time(), rospy.Duration(60.0))
        rospy.on_shutdown(self.shutdown)
        self.map = costMap
        #self.trans = Point(.0, .0, .0)
        #self.rot = Quaternion(.0, .0, .0, 1.0)
        self.front = 0
        self.pos_init = 1
        self.x = []
        self.y = []

    def run(self):
        rospy.spin()
    #def callback_pose(self, data):
    #    self.trans = Point(data.position.x, data.position.y, data.position.z)
    #    self.rot =  Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    #    self.pos_init = 1
    def callback_scan(self, data):
        if self.pos_init == 0:
            return 
        d = np.array(data.ranges)
        self.front = np.mean(d[315:325]) < 0.8
        try:
            (trans,rot) = self.listener.lookupTransform('visual_odom', 'base_footprint', rospy.Time.now())
            Rt = np.dot(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))
            new_path = self.map.toMapXY(trans[0], trans[1])
            self.x.append(new_path[0])
            self.y.append(new_path[1])

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
