import numpy as np
# import matplotlib.pyplot as plt

import current_types

class Map:
    """docstring for Map"""
    def __init__(self, map_msg):
        self.array = self.update_grid(map_msg.data)
        self.height = map_msg.info.height #Ngridpoints
        self.width = map_msg.info.width #Ngridpoints
        self.res = map_msg.info.resolution #m/cell
        self.origin = map_msg.info.origin
        self.pos = [self.origin.position.x, self.origin.position.y]
        self.grid = np.asarray(self.array, dtype=np.int8).reshape(self.height, self.width)
        self.current = current_types.Current("Sine Waves Horiz", np.zeros_like(self.grid,dtype="float"))
        # self.current = self.generate_current(np.zeros_like(self.grid))

    def update_grid(self, grid):
        grid = list(grid)
        for i,value in enumerate(grid):
            if value < 0:
                grid[i] = 1
            else:
                grid[i] = value/100.
        return grid

    def risk_at(self,p):
        x_cell = int((p[0] - self.pos[0])/self.res)
        y_cell = int((p[1] - self.pos[1])/self.res)
        return self.grid[x_cell, y_cell]

    def current_at(self,p):
        x_cell = int((p[0] - self.pos[0])/self.res)
        y_cell = int((p[1] - self.pos[1])/self.res)
        return 0.0,0.0
        # return self.current.current_x[x_cell, y_cell], self.current.current_y[x_cell, y_cell]


class MapMsg():
    def __init__(self,info,data):
        self.info = info
        self.data = data

class Info():
    def __init__(self,height,width,res,origin):
        self.height = height
        self.width = width
        self.resolution = res
        self.origin = origin

class Origin():
    def __init__(self,position):
        self.position = position

class Position():
    def __init__(self, x, y):
        self.x = x
        self.y = y

# p = Position(0,0)
# o = Origin(p)
# h = 100
# w = 100
# r = .1
# i = Info(h,w,r,o)
# d = np.random.rand(w*h)*100
# mm = MapMsg(i,d)
# m = Map(mm)
# x = np.arange(w)
# y = np.arange(h)
# c_x = m.current.current_x
# c_y = m.current.current_y
# plt.streamplot(x,y,c_x,c_y)
# plt.show()
