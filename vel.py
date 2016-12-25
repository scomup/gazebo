#! /usr/bin/env python
import rospy
import roslib
#roslib.load_manifest('your_package')
from geometry_msgs.msg import Twist
rospy.init_node('dummy')
# publish to cmd_vel
p = rospy.Publisher('/cmd_vel', Twist,queue_size = 10)
# create a twist message, fill in the details
twist = Twist()
twist.linear.x =0.0;                 
twist.linear.y = 0.0; 
twist.linear.z = 0.0;   
twist.angular.x = 0; 
twist.angular.y = 0;  
twist.angular.z = 0; 
while True:
    p.publish(twist)
