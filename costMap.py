#!/usr/bin/env python
# coding:utf-8

import numpy as np

class costMap():
    def __init__(self, row = 100, col = 100, c_row = 50, c_col = 50, resolution = 0.2):
        self.map_data = np.zeros((row,col))
        self.c_row = c_row
        self.c_col = c_col
        self.row = row
        self.col = col
        self.resolution = resolution

    def toMapXY(self, x, y):
        return (round(self.c_row + x / self.resolution), round(self.c_col + y / self.resolution))          
    def toWorldXY(self, x, y):
        return (float(self.resolution * (x - self.c_row)), float(self.resolution * (y - self.c_col)))         


    def setCostMap(self, x, y, cost):
        self.map_data[x, y] = cost

    def getCostMap(self, x, y):
        return self.map_data[x, y]

    def getNearBy(self, x, y):
        tmp = []
        cx = x
        cy = y
        for i in range(-1,2):
            for j in range(-1,2):
                if i == 0 and j == 0:
                    continue
                nx = int(cx + i)
                ny = int(cy + j)
                if (nx >= 0 and nx < self.row) or (ny >= 0 and ny < self.col):
                    tmp.append((nx, ny))
        return tmp

    def setVisit(self , x, y):
        x_grid, y_grid = np.meshgrid(np.arange(9), numpy.arange(9)) 
        disk = ((x_grid-4)**2 + (y_grid-4)**2) <= radius**2
        p = self.toMapXY(x, y)
        self.setCostMap(p[0], p[1], 1)
        nearby = self.getNearBy(p[0], p[1])
        for i in nearby:
            cost = self.getCostMap(i[0], i[1])
            if cost != 2:
                self.setCostMap(i[0], i[1], 1)
