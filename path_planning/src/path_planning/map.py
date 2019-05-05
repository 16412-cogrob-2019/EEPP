import numpy as np
import current

class Map():
    """docstring for Map"""
    def __init__(self, map_msg):
        self.array = self.update_grid(map_msg.data)
        self.height = map_msg.info.height
        self.width = map_msg.info.width
        self.res = map_msg.info.resolution #m/cell
        self.origin = map_msg.info.origin
        self.pos = [self.origin.position.x, self.origin.position.y]
        self.grid = np.asarray(self.array, dtype=np.int8).reshape(self.height, self.width)
        self.current = Current("Sine Waves Horiz", np.zeros_like(self.grid))
        # self.current = self.generate_current(np.zeros_like(self.grid))


    def update_grid(self, grid):
        for i,value in enumerate(grid):
            if value < 0:
                grid[i] = 1
            else:
                grid[i] = value/100.
        return grid

    def risk(p):
        x_cell = p[0]/self.res - self.pos[0]
        y_cell = p[1]/self.res - self.pos[1]
        return self.grid[x_cell, y_cell]

    def current(p):
        x_cell = p[0]/self.res - self.pos[0]
        y_cell = p[1]/self.res - self.pos[1]
        return self.current.current_x[x_cell, y_cell], self.current.current_y[x_cell, y_cell]

