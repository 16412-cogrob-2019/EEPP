import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage

import current_types

class MapObj:
    """docstring for Map"""
    def __init__(self, map_msg, add_blur=True):
        # self.array = self.update_grid(map_msg.data)
        # self.height = map_msg.info.height #Ngridpoints
        # self.width = map_msg.info.width #Ngridpoints
        # self.res = map_msg.info.resolution #m/cell
        self.res = 1#0.050000
        self.height, self.width = map_msg.shape
        # self.origin = map_msg.info.origin
        # self.pos = [self.origin.position.x, self.origin.position.y]
        self.pos = [0.000000, 0.000000]
        # self.grid = np.asarray(self.array, dtype=np.int8).reshape(self.height, self.width)
        self.grid = map_msg
        # self.add_obstacle()
        self.risk = self.get_risk_field(self.grid, add_blur)
        self.current = current_types.Current("Counterclockwise Whirlpool", 2, np.zeros_like(self.grid,dtype="float"))

    def update_grid(self, grid):
        grid = list(grid)
        for i,value in enumerate(grid):
            if value < 0:
                grid[i] = 1
            else:
                grid[i] = value/100.
        return grid

    def add_obstacle(self):
        x = np.arange(self.width)
        y = np.arange(self.height)
        grid = np.ones_like(self.grid)
        grid[x>51,:] = 0
        grid[x<49,:] = 0
        grid[:,y>51] = 0
        grid[:,y<49] = 0
        self.grid = grid
        print "add_obstacle"

    def get_risk_field(self, grid, add_blur):
        if add_blur:
            std_dev = 0.1
            scaling = 2
            sigma = (std_dev/self.res, std_dev/self.res)
            blurred_grid = scipy.ndimage.gaussian_filter(grid.astype(float), sigma, mode='reflect', cval=0.0, truncate=4.0)
            blurred_grid = np.clip(blurred_grid*scaling, 0, 1)
            # xx, yy = np.meshgrid(range(0,self.width), range(0,self.height))
            # print(blurred_grid)
            # plt.scatter(xx,yy, np.maximum(blurred_grid, grid))
            # plt.show()
            return np.maximum(blurred_grid, grid)
        else:
            return grid

    def get_cell_pos(self, p):
        x_cell = int((p[0] - self.pos[0])/self.res)
        y_cell = int((p[1] - self.pos[1])/self.res)
        return x_cell, y_cell

    def risk_at(self, p, use_blur = False):
        y_cell, x_cell = self.get_cell_pos(p)
        #x_cell, y_cell = self.get_cell_pos(p)
        return self.risk[x_cell, y_cell]

    def current_at(self, p):
        y_cell, x_cell = self.get_cell_pos(p)
        return self.current.current_x[x_cell, y_cell], self.current.current_y[x_cell, y_cell]

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

# CURRENTS
# if current_type == "Sine Waves Horiz":
#     return self.sine_wave_h(grid, strength)

# if current_type == "Sine Waves Vert":
#     return self.sine_wave_v(grid, strength)

# if current_type == "Inverted Sine Vert":
#     return self.inv_sine_v(grid, strength)

# if current_type == "Inverted Sine Horiz":
#     return self.inv_sine_h(grid, strength)

# if current_type == "Clockwise Whirlpool":
#     return self.whirlpool_cw(grid, strength)

# if current_type == "Counterclockwise Whirlpool":
#     return self.whirlpool_ccw(grid, strength)

# if current_type == "Hyperbola":
#     return self.hyperbola(grid, strength)

# if current_type == "Sink Hole":
#     return self.sink_hole(grid, strength)

# if current_type == "Source":
#     return self.source(grid, strength)

# if current_type == "Right Horiz":
#     return self.horizontal_r(grid, strength)

# if current_type == "Up Vert":
#     return self.vertical_u(grid, strength)

# if current_type == "Left Horiz":
#     return self.horizontal_l(grid, strength)

# if current_type == "Down Vert":
#     return self.vertical_d(grid, strength)
