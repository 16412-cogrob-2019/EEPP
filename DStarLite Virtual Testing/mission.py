import matplotlib.pyplot as plt
from numpy import dtype
from Astar import astar

# class for defining an underwater exploration mission
class MISSION:
    def __init__(self, discretization_distance=1, worldsize_x=100, worldsize_y=100):
        """
        discretization_distance = discretization distance in meters
        worldsize_x, worldsize_y = worldsizes in meters
        """
        self.worldsize_x = worldsize_x
        self.worldsize_y = worldsize_y
        self.discretization = int(round(worldsize_x / discretization_distance))+1
