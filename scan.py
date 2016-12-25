#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

def callback(data):
    #r = np.arange(-135, 135, 270./640.) * math.pi/180.
    d = np.array(data.ranges)
    left  = np.mean(d[100:110])
    right = np.mean(d[530:540])
    front = np.mean(d[315:325])
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
