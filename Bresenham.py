#!/usr/bin/env python
# coding:utf-8
from matplotlib import pyplot as plt
import numpy as np
def get_line(image, start, end):
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    x1 = int(x1)
    x2 = int(x2)
    y1 = int(y1)
    y2 = int(y2)
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    points.pop()
    for p in points:
        if image[p[0], p[1]] != 2:
            image[p[0], p[1]] = 1

#cmap = np.zeros((50,50))
#print get_line(cmap, (1,1),(3,40))
#fig, ax = plt.subplots()
#ax.set_aspect('equal')
#plt.axis([0,50,0,50])
#ax.pcolor(cmap,cmap=plt.cm.Reds,edgecolors='k')
#plt.show()

