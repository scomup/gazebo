import numpy as np
import time
import threading
import operator
import grid_map as gm

class dstar_planner(threading.Thread):
    def __init__(self,grid_map):
        threading.Thread.__init__(self)
        self.open = []
        self.grid_map = grid_map
    def add_start(self,cell):
        self.insert(cell,0)
    def insert(self,cell,h_new):
        if cell.tag == gm.Tag.NEW:
            cell.k = h_new
        else:
            cell.k = min(h_new,cell.k)
        cell.h = h_new
        if cell.tag != gm.Tag.OPEN:
            cell.tag = gm.Tag.OPEN
            self.open.append(cell)
        self.open.sort(key=operator.attrgetter('k'))
    def neighbor(self,cell):
        tmp = []
        rows = len(self.grid_map.map_data)
        cols = len(self.grid_map.map_data[0])
        for i in range(-1,2):
            for j in range(-1,2):
                if (cell.row + i < rows ) and (cell.row + i >= 0 ) and (cell.col + j < cols ) and (cell.col + j >= 0 ): 
                    if i == 0 and j == 0:
                        continue
                    tmp.append(self.grid_map.map_data[cell.row + i][cell.col + j])
        return tmp
    def process_state(self):
        if len(self.open) == 0:
            return -1
        #self.open.sort(key=operator.attrgetter('k'))
        cell_current = self.open[0]
        if cell_current.stat ==  gm.Stat.OBSTACLE:
            return -1
        del self.open[0]
        cell_current.tag = gm.Tag.CLOSE
        k_old = cell_current.k
        neighbor = self.neighbor(cell_current)
        if k_old < cell_current.h:
            for cell in neighbor:
                if cell.tag == gm.Tag.NEW and cell.h < k_old and cell_current.h > cell_current.cost(cell) + cell.h:
                    cell_current.back_point = cell
                    cell_current.h = cell_current.cost(cell) + cell.h
        if k_old == cell_current.h:
            for cell in neighbor:
                if cell.tag == gm.Tag.NEW or (cell.back_point == cell_current and cell.h != cell.x + cell_current.cost(cell) + cell_current.h) or (cell.back_point != cell_current and cell.h >  cell_current.cost(cell) + cell_current.h):
                    cell.back_point = cell_current
                    self.insert(cell, cell_current.cost(cell) + cell_current.h)
                    
        return k_old
    def init_plan(self):
        k = 0
        while k != -1:
            k = self.process_state()
