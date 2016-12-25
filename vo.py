import numpy as np
from matplotlib import pyplot as plt
import tf
import threading
import rospy
import sys
import time
import random
import math
from sensor_msgs.msg import LaserScan


class mapDrawer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.v_data = np.zeros((100,100))
        
    def setPoint(self,x,y,c):
        self.px = int(50+x)
        self.py = int(50+y)
        self.v_data[self.px,self.py] = c
        self.ax.set_aspect('equal')

    def run(self):
        fig,self.ax=plt.subplots()
        self.ax.invert_yaxis()
        #plt.axis([-15,15,-15,15])
        while True:
            self.ax.pcolor(self.v_data,cmap=plt.cm.Reds,edgecolors='k')
            plt.pause(0.1)
 

class scanListener(threading.Thread):
    def __init__(self):  
        threading.Thread.__init__(self)
        rospy.init_node('listener', anonymous=True) 
        rospy.Subscriber("scan", LaserScan, self.callback)
    def callback(self, data):
        d = np.array(data.ranges)
        self.right  = np.mean(d[100:110]) < 0.3
        self.left = np.mean(d[530:540]) < 0.3
        self.front = np.mean(d[315:325]) < 0.3
    def run(self):
         rospy.spin()

def direction(yew):
    if -math.pi/6 < yew and yew <= math.pi/6:
        R = np.matrix([[1,0],[0,1]])
        f =  0
        return R, f
    elif math.pi/2 - math.pi/12 < yew and yew <= math.pi/2 + math.pi/12:
        R = np.matrix([[0,-1],[1,0]])
        f =  1
        return R, f
    elif math.pi - math.pi/12 < yew or yew <= -math.pi + math.pi/12:
        R = np.matrix([[-1,0],[0,-1]])
        f =  2
        return R, f
    elif -math.pi/2 - math.pi/12 < yew and yew <= -math.pi/2 + math.pi/12:
        R = np.matrix([[0,1],[-1,0]])
        f =  3
        return R, f
    else:
        R = np.matrix([[0,0],[0,0]])
        f =  4
        return R, f


if __name__ == '__main__':
    scaner = scanListener()
    listener = tf.TransformListener()
    drawer = mapDrawer()
    drawer.start()
    scaner.start()
    rospy.sleep(1.0)


    while True:
        time.sleep(0.1)
        (trans,rot) = listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        yaw = euler[2]
        x = int(trans[0] * 4)
        y = int(trans[1] * 4)
        R,f = direction(yaw)
        drawer.setPoint(x, y, 2)
        if f == 4:
            continue
        if(scaner.left):
            pose = np.matrix([[0],[1]])
            pose = R * pose
            drawer.setPoint(x + pose[0,0], y + pose[1,0], 1)
        if(scaner.right):
            pose = np.matrix([[0],[-1]])
            pose = R * pose
            drawer.setPoint(x + pose[0,0], y + pose[1,0], 1)
        if(scaner.front):
            pose = np.matrix([[1],[0]])
            pose = R * pose
            drawer.setPoint(x + pose[0,0], y + pose[1,0], 1)

    
