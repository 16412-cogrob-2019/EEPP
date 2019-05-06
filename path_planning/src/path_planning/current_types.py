import numpy as np
class Current():
    def __init__(self, current_type, grid):
        self.type = current_type
        self.height_cells = grid.shape[1]
        self.width_cells = grid.shape[0]
        self.current_x, self.current_y = self.generate_current(grid, current_type)

    def generate_current(self, grid, current_type):
        if current_type == "Sine Waves Horiz":
            return self.sine_wave_h(grid)

        if current_type == "Sine Waves Vert":
            return self.sine_wave_v(grid)

        if current_type == "Inverted Sine Vert":
            return self.inv_sine_v(grid)

        if current_type == "Inverted Sine Horiz":
            return self.inv_sine_h(grid)

        if current_type == "Clockwise Whirlpool":
            return self.whirlpool_cw(grid)

        if current_type == "Counterclockwise Whirlpool":
            return self.whirlpool_ccw(grid)

        if current_type == "Hyperbola":
            return self.hyperbola(grid)

        if current_type == "Sink Hole":
            return self.sink_hole(grid)

        if current_type == "Source":
            return self.source(grid)

        if current_type == "Right Horiz":
            return self.horizontal_r(grid)

        if current_type == "Up Vert":
            return self.vertical_u(grid)

        if current_type == "Left Horiz":
            return self.horizontal_l(grid)

        if current_type == "Down Vert":
            return self.vertical_d(grid)

        return grid, np.zeros_like(grid)

    def sine_wave_h(self, grid):
        c_x = np.zeros_like(grid,dtype="float")
        c_y = np.zeros_like(c_x,dtype="float")
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                col_transformed = float(col)/self.width_cells * 6*np.pi
                c_x[row, col] = abs(np.sin(col_transformed))
                c_y[row, col] = np.cos(col_transformed)
        return c_x, c_y

    def sine_wave_v(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                row_transformed = float(row)/self.width_cells * 6*np.pi
                c_x[row, col] = np.sin(row_transformed)
                c_y[row, col] = abs(np.cos(row_transformed))
        return c_x, c_y

    def inv_sine_h(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                col_transformed = float(col)/self.width_cells * 6*np.pi
                c_x[row, col] = np.sin(col_transformed)
                c_y[row, col] = abs(np.cos(col_transformed))
        return c_x, c_y

    def inv_sine_v(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                row_transformed = float(row)/self.width_cells * 6*np.pi
                c_x[row, col] = abs(np.sin(row_transformed))
                c_y[row, col] = np.cos(row_transformed)
        return c_x, c_y

    def whirlpool_cw(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        mid = np.array([int(self.width_cells/2), int(self.height_cells/2)])
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                diff = mid-np.array([row,col])
                if np.linalg.norm(diff) == 0:
                    amplitude = 0
                else:
                    amplitude = 1./np.linalg.norm(diff)
                c_x[row, col] = -amplitude*np.cos(np.arctan2(diff[1],diff[0]))
                c_y[row, col] = amplitude*np.sin(np.arctan2(diff[1],diff[0]))
        return c_x, c_y

    def whirlpool_ccw(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        mid = np.array([int(self.width_cells/2), int(self.height_cells/2)])
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                diff = mid-np.array([row,col])
                if np.linalg.norm(diff) == 0:
                    amplitude = 0
                else:
                    amplitude = 1./np.linalg.norm(diff)
                c_x[row, col] = amplitude*np.cos(np.arctan2(diff[1],diff[0]))
                c_y[row, col] = -amplitude*np.sin(np.arctan2(diff[1],diff[0]))
        return c_x, c_y

    def hyperbola(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        mid = np.array([int(self.width_cells/2), int(self.height_cells/2)])
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                diff = mid-np.array([row,col])
                if np.linalg.norm(diff) == 0:
                    amplitude = 0
                else:
                    amplitude = 1./np.linalg.norm(diff)
                c_x[row, col] = amplitude*np.cos(np.arctan2(diff[1],diff[0]))
                c_y[row, col] = amplitude*np.sin(np.arctan2(diff[1],diff[0]))
        return c_x, c_y

    def sink_hole(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        mid = np.array([int(self.width_cells/2), int(self.height_cells/2)])
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                diff = mid-np.array([row,col])
                if np.linalg.norm(diff) == 0:
                    amplitude = 0
                else:
                    amplitude = 1./np.linalg.norm(diff)
                c_x[row, col] = amplitude*np.sin(np.arctan2(diff[1],diff[0]))
                c_y[row, col] = amplitude*np.cos(np.arctan2(diff[1],diff[0]))
        return c_x, c_y

    def source(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        mid = np.array([int(self.width_cells/2), int(self.height_cells/2)])
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                diff = mid-np.array([row,col])
                if np.linalg.norm(diff) == 0:
                    amplitude = 0
                else:
                    amplitude = 1./np.linalg.norm(diff)
                c_x[row, col] = -amplitude*np.sin(np.arctan2(diff[1],diff[0]))
                c_y[row, col] = -amplitude*np.cos(np.arctan2(diff[1],diff[0]))
        return c_x, c_y

    def horizontal_l(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                c_x[row, col] = -1
                c_y[row, col] = 0
        return c_x, c_y

    def horizontal_r(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                c_x[row, col] = 1
                c_y[row, col] = 0
        return c_x, c_y

    def vertical_d(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                c_x[row, col] = 0
                c_y[row, col] = -1
        return c_x, c_y

    def vertical_u(self, grid):
        c_x = grid
        c_y = np.zeros_like(c_x)
        for col in range(self.height_cells):
            for row in range(self.width_cells):
                c_x[row, col] = 0
                c_y[row, col] = 1
        return c_x, c_y
