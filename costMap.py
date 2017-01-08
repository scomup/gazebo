#!/usr/bin/env python
# coding:utf-8

OBSTACLE = 2
VISITED = 1
FREE = 0
import numpy as np
import ConfigParser


class costMap():
    def __init__(self, row = 100, col = 100, c_row = 50, c_col = 50, resolution = 0.2):
        inifile = ConfigParser.SafeConfigParser()
        inifile.read('./config.ini')
        self.safe_move_base_distance_go = inifile.getfloat('path_planner', 'safe_move_base_distance_go')
        self.safe_move_base_distance_turn = inifile.getfloat('path_planner', 'safe_move_base_distance_turn')
        self.map_data = np.zeros((row,col))
        self.c_row = c_row
        self.c_col = c_col
        self.row = row
        self.col = col
        self.resolution = resolution
        self.nearbyRatioH = int(1/resolution/3)
        self.nearbyRatio2 = self.nearbyRatioH * 2 + 1
        x_grid, y_grid = np.meshgrid(np.arange(self.nearbyRatio2), np.arange(self.nearbyRatio2)) 
        self.mask = ((x_grid-self.nearbyRatioH)**2 + (y_grid-self.nearbyRatioH)**2) <= self.nearbyRatioH**2

    def check_obs(self, x, y):
        p = self.toMapXY(x, y)
        return self.map_data[p[0], p[1]] == OBSTACLE

    def get_goal(self, x, y, d):
        p = self.toMapXY(x, y)
        if(d == 'up'):
            line = np.copy(self.map_data[:, p[1]])
            line[0:p[0]] = FREE
            idx = np.where(line == OBSTACLE)[0]
            if idx.size == 0:
                return (x + 5.0, y)
            else: 
                return ((np.min(idx) -  self.c_row) * self.resolution - self.safe_move_base_distance_go, y)
        elif(d == 'down'):
            line = np.copy(self.map_data[:, p[1]])
            line[p[0]:] = FREE
            idx = np.where(line == OBSTACLE)[0]
            if idx.size == 0:
                return (x - 5.0, y)
            else: 
                return ((np.max(idx) -  self.c_row) * self.resolution + self.safe_move_base_distance_go, y)
        elif(d == 'left'):
            line = self.map_data[p[0], :]
            idx = np.where(line == OBSTACLE)[0]
            if idx.size == 0:
                return (x, y + self.safe_move_base_distance_turn)
            else: 
                #a = np.min(idx) -  self.c_row) * self.resolution -0.3
                #b =  a - x - 0.3 
                return (x, y + self.safe_move_base_distance_turn)
        elif(d == 'right'):
            line = self.map_data[p[0], :]
            idx = np.where(line == 2)[0]
            if idx.size == 0:
                return (x, y - self.safe_move_base_distance_turn)
            else: 
            #    a = np.max(idx) -  self.c_row) * self.resolution -0.3
            #    b =  a - x + 0.3 
                return (x, y - self.safe_move_base_distance_turn)
        else:
            print 'Error'


    def toMapXY(self, x, y):
        return (round(self.c_row + x / self.resolution), round(self.c_col + y / self.resolution))          
    def toWorldXY(self, x, y):
        
        return (float(self.resolution * (x - self.c_row)), float(self.resolution * (y - self.c_col)))         


    def setCostMap(self, x, y, cost):
        self.map_data[x, y] = cost
        self.map_data[(x - 2):][:5][:, (y - 2):][:, :5] = cost


    def getCostMap(self, x, y):
        return self.map_data[x, y]


    def setVisit(self , x, y):
        p = self.toMapXY(x, y)
        data = self.map_data[(p[0] - self.nearbyRatioH):][:self.nearbyRatio2][:, (p[1] - self.nearbyRatioH):][:, :self.nearbyRatio2]
        mask2 = (data != OBSTACLE)
        mask = mask2 * self.mask
        self.map_data[(p[0] - self.nearbyRatioH):][:self.nearbyRatio2][:, (p[1] - self.nearbyRatioH):][:, :self.nearbyRatio2] = np.logical_not(mask) * data + mask * VISITED
