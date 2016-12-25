from enum import Enum
import numpy as np
import matplotlib.pyplot as plt

INF = float("inf")
INF = 200
BIG = 100
BIG_2 = BIG/2
class Stat(Enum):
    FREE = 0
    OBSTACLE = 1

class Tag(Enum):
    NEW = 0
    OPEN = 1
    CLOSE = 2

class map_cell:
    def __init__(self,row,col):
        
        self.back_point = self
        self.path = False
        self.tag = Tag.NEW
        self.stat = Stat.FREE
        self.h = 0
        self.k = 0
        self.weight = 0
        self.row = row
        self.col = col

    def cost(self,to_ceil):                  
        a = abs(to_ceil.row - self.row)
        b = abs(to_ceil.col - self.col)
        if a > 1 or b > 1:
            print 'COST ERROR'
            return -1
        elif to_ceil.stat  == Stat.OBSTACLE or self.stat == Stat.OBSTACLE:
            return INF
        elif a == 0 or b == 0:
            return 1
        else:
            return 1.4

class grid_map:
    def __init__(self,rows,cols):
        self.rows = rows
        self.cols = cols
        self.map_data = []
        self.v_data = np.zeros((rows, cols))
        self.v_data.fill(Stat.FREE)
        for i in range(rows):
            line = []
            for j in range(cols):
                cell = map_cell(i,j)
                line.append(cell)
            self.map_data.append(line)

    def nearby(self,cell,radius):
        row_l = max(cell.row - radius, 0)
        row_h = min(cell.row + radius + 1, self.rows )
        col_l = max(cell.col - radius, 0)
        col_h = min(cell.col + radius + 1, self.cols )
        tmp = []
        for i in range(row_l, row_h):
            tmp.extend(self.map_data[i][col_l : col_h])
        #del tmp[(radius*2+1)*radius + radius]
        return tmp
    def distance(self,cell_a,cell_b):
        a = abs(cell_a.row - cell_b.row) 
        b = abs(cell_a.col - cell_b.col)
        return max(a, b)          
    def update_weight(self,radius):
        for i in range(self.rows):
            for j in range(self.cols):
                cell_current = self.map_data[i][j]
                if cell_current.stat == Stat.OBSTACLE: 
                    cell_current.weight = BIG
                    # update v for visulization
                    self.v_data[cell_current.row,cell_current.col] = cell_current.weight
                    nearby = self.nearby(cell_current,2 * radius + 1)
                    for cell in nearby:
                        d = self.distance(cell_current, cell)
                        if d <=  radius:
                            weight = BIG_2
                        else:
                            weight = 8 - d
                        cell.weight = max(cell.weight, weight)
                        # update v for visulization
                        self.v_data[cell.row,cell.col] = cell.weight                      
        

    def set_goal(self,row,col):
        self.goal = self.map_data[row][col]
        
    def set_start(self,row,col):
        self.start = self.map_data[row][col]

    def init_map(self):
        self.map_data[0 ][20].stat = Stat.OBSTACLE
        self.map_data[1 ][20].stat = Stat.OBSTACLE
        self.map_data[2 ][20].stat = Stat.OBSTACLE
        self.map_data[3 ][20].stat = Stat.OBSTACLE
        self.map_data[4 ][20].stat = Stat.OBSTACLE
        self.map_data[5 ][20].stat = Stat.OBSTACLE
        self.map_data[6 ][20].stat = Stat.OBSTACLE
        self.map_data[7 ][20].stat = Stat.OBSTACLE
        self.map_data[8 ][20].stat = Stat.OBSTACLE
        self.map_data[9 ][20].stat = Stat.OBSTACLE
        self.map_data[10][20].stat = Stat.OBSTACLE
        self.map_data[11][20].stat = Stat.OBSTACLE
        self.map_data[12][20].stat = Stat.OBSTACLE
        self.map_data[13][20].stat = Stat.OBSTACLE
        self.map_data[14][20].stat = Stat.OBSTACLE
        self.map_data[15][20].stat = Stat.OBSTACLE
        self.map_data[16][20].stat = Stat.OBSTACLE
        self.map_data[17][20].stat = Stat.OBSTACLE
        self.map_data[18][20].stat = Stat.OBSTACLE
        self.map_data[19][20].stat = Stat.OBSTACLE
        self.map_data[20][20].stat = Stat.OBSTACLE      
        self.map_data[40][0 ].stat = Stat.OBSTACLE
        self.map_data[40][1 ].stat = Stat.OBSTACLE
        self.map_data[40][2 ].stat = Stat.OBSTACLE
        self.map_data[40][3 ].stat = Stat.OBSTACLE
        self.map_data[40][4 ].stat = Stat.OBSTACLE
        self.map_data[40][5 ].stat = Stat.OBSTACLE
        self.map_data[40][6 ].stat = Stat.OBSTACLE
        self.map_data[40][7 ].stat = Stat.OBSTACLE
        self.map_data[40][8 ].stat = Stat.OBSTACLE
        self.map_data[40][9 ].stat = Stat.OBSTACLE
        self.map_data[40][10].stat = Stat.OBSTACLE
        self.map_data[40][11].stat = Stat.OBSTACLE
        self.map_data[40][12].stat = Stat.OBSTACLE
        self.map_data[40][13].stat = Stat.OBSTACLE
        self.map_data[40][14].stat = Stat.OBSTACLE
        self.map_data[40][15].stat = Stat.OBSTACLE
        self.map_data[40][16].stat = Stat.OBSTACLE
        self.map_data[40][17].stat = Stat.OBSTACLE
        self.map_data[40][18].stat = Stat.OBSTACLE
        self.map_data[40][19].stat = Stat.OBSTACLE
        self.map_data[40][20].stat = Stat.OBSTACLE
        self.map_data[40][21].stat = Stat.OBSTACLE
        self.map_data[40][22].stat = Stat.OBSTACLE
        self.map_data[40][23].stat = Stat.OBSTACLE
        self.map_data[40][24].stat = Stat.OBSTACLE
        self.map_data[40][25].stat = Stat.OBSTACLE
        self.map_data[40][26].stat = Stat.OBSTACLE
        self.map_data[40][27].stat = Stat.OBSTACLE
        self.map_data[40][28].stat = Stat.OBSTACLE
        self.map_data[40][29].stat = Stat.OBSTACLE
        self.map_data[40][30].stat = Stat.OBSTACLE
        self.map_data[40][31].stat = Stat.OBSTACLE
        self.map_data[40][32].stat = Stat.OBSTACLE
        self.map_data[40][33].stat = Stat.OBSTACLE
        self.map_data[40][34].stat = Stat.OBSTACLE
        self.map_data[40][35].stat = Stat.OBSTACLE
        self.map_data[40][36].stat = Stat.OBSTACLE
        self.map_data[40][37].stat = Stat.OBSTACLE
        self.map_data[40][38].stat = Stat.OBSTACLE
        self.map_data[40][39].stat = Stat.OBSTACLE
        self.map_data[40][40].stat = Stat.OBSTACLE    
        self.map_data[10][40].stat = Stat.OBSTACLE
        self.map_data[11][40].stat = Stat.OBSTACLE
        self.map_data[12][40].stat = Stat.OBSTACLE
        self.map_data[13][40].stat = Stat.OBSTACLE
        self.map_data[14][40].stat = Stat.OBSTACLE
        self.map_data[15][40].stat = Stat.OBSTACLE
        self.map_data[16][40].stat = Stat.OBSTACLE
        self.map_data[17][40].stat = Stat.OBSTACLE
        self.map_data[18][40].stat = Stat.OBSTACLE
        self.map_data[19][40].stat = Stat.OBSTACLE
        self.map_data[20][40].stat = Stat.OBSTACLE
        self.map_data[21][40].stat = Stat.OBSTACLE
        self.map_data[22][40].stat = Stat.OBSTACLE
        self.map_data[23][40].stat = Stat.OBSTACLE
        self.map_data[24][40].stat = Stat.OBSTACLE
        self.map_data[25][40].stat = Stat.OBSTACLE
        self.map_data[26][40].stat = Stat.OBSTACLE
        self.map_data[27][40].stat = Stat.OBSTACLE
        self.map_data[28][40].stat = Stat.OBSTACLE
        self.map_data[29][40].stat = Stat.OBSTACLE
        self.map_data[30][40].stat = Stat.OBSTACLE
        self.map_data[31][40].stat = Stat.OBSTACLE
        self.map_data[32][40].stat = Stat.OBSTACLE
        self.map_data[33][40].stat = Stat.OBSTACLE
        self.map_data[34][40].stat = Stat.OBSTACLE
        self.map_data[35][40].stat = Stat.OBSTACLE
        self.map_data[36][40].stat = Stat.OBSTACLE
        self.map_data[37][40].stat = Stat.OBSTACLE
        self.map_data[38][40].stat = Stat.OBSTACLE
        self.map_data[39][40].stat = Stat.OBSTACLE


    