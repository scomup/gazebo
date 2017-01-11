#!/usr/bin/env python
# coding:utf-8

import rospy
from costMap import costMap
import scanListener1
import scanListener
from mapDrawer import mapDrawer
from explorer import explorer
from pathPlanner import pathPlanner


def explorer_demo():
    try:
        rospy.init_node('plan_planner', anonymous=False)
        cmap = costMap(row = 200, col = 200, c_row = 40, c_col = 40, resolution = 0.05)
        scan = scanListener1.scanListener(cmap)
        scan.setDaemon(True)
        scan.start()
        drawer = mapDrawer()
        planer = explorer(cmap, scan, drawer)
        planer.setDaemon(True)
        planer.start()
        drawer.run()
    except:
        print 'Main except!'  

def pathPlanner_demo():
    try:
        rospy.init_node('plan_planner', anonymous=False)
        cmap = costMap(row = 200, col = 200, c_row = 40, c_col = 40, resolution = 0.05)
        scan = scanListener.scanListener(cmap)
        scan.setDaemon(True)
        scan.start()
        drawer = mapDrawer()
        planer = pathPlanner(cmap, scan, drawer)
        planer.setDaemon(True)
        planer.start()
        drawer.run()
    except:
        print 'Main except!'

if __name__ == '__main__':
    #pathPlanner_demo()
    explorer_demo()
            
            
