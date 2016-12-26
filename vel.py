#! /usr/bin/env python
import rospy
import roslib
import numpy as np
import time
import math
import tf

#roslib.load_manifest('your_package')
from geometry_msgs.msg import Twist
rospy.init_node('dummy')
# publish to cmd_vel
p = rospy.Publisher('/Rulo/cmd_vel', Twist,queue_size = 10)
# create a twist message, fill in the details
twist = Twist()
cmd  = 0.1 * np.array([0,1])
#np.array([0,1])
#np.array([0,-1])
front =  np.array([1, 0])
turn_left  =  np.array([0, 1])
turn_right =  np.array([0,-1])
go_left = np.array([1,1])
go_right = np.array([1,-1])
stop  =  np.array([0,0])
cmd   =  0.1 * np.array([0,-1])
twist.linear.x = cmd[0]
twist.angular.z = cmd[1]


#while True:
 #   p.publish(twist)

listener = tf.TransformListener()



goal =  np.array([0,1])
time.sleep(0.1)
check_orientation = 1 
while True:
    time.sleep(0.1)
    (trans,rot) = listener.lookupTransform('/Rulo/odom', 'Rulo/base_link', rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)

    current_theta = euler[2]
    current = np.array([trans[0], trans[1]])
    v1 = goal - current
    dist = np.linalg.norm(v1)
    a = v1[1]
    b = v1[0]
    if b  > 0 and a > 0:
        goal_theta = math.atan(a/b)
    elif a  > 0 and b < 0:
        goal_theta = - math.pi - math.atan(a/b)
    elif a  < 0 and b > 0:
        goal_theta = math.atan(a/b)
    else:
        goal_theta = math.atan(a/b)  - math.pi
    
    #print trans[0] , trans[1]
    #print  goal_theta
    if dist > 0.3:


        #if np.sign(current_theta) == np.sign(goal_theta):
        diff_theta =  current_theta - goal_theta
        #elif np.sign(current_theta) == -1 and np.sign(goal_theta) == 1:
        #    diff_theta = - (goal_theta + current_theta + math.pi) 
        #else:
        #    diff_theta = + (goal_theta + current_theta + math.pi)
        if diff_theta < -math.pi:
            diff_theta = diff_theta + 2* math.pi


        
        print current_theta, goal_theta, diff_theta,dist

        if abs(diff_theta) > math.pi / 30 and check_orientation:
            if( diff_theta < 0):
                cmd = turn_left
            else:
                cmd = turn_right
        else:

            cmd = front
            check_orientation = 0

                #print front
        #print diff_theta2
        #cmd = stop
    else:
        cmd = stop
    #cmd = stop
    cmd  = 0.1 * cmd
    twist.linear.x = cmd[0]
    twist.angular.z = cmd[1]

    p.publish(twist)

            
            
