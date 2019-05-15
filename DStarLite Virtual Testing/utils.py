import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

"""
Return the distance from node1 to node2
"""
def dist(node1, node2):
    return (((node1.position[0] - node2.position[0]) ** 2) + ((node1.position[1] - node2.position[1]) ** 2)) **0.5

def InspectReefData3D():
    def StandardReefFunction(x, y):
        x = (x-0.5 *100)/15
        y = (y-0.5 *100)/15
        return (1.0 - x / 2.0 + x ** 5.0 + y ** 3.0) * np.exp(-x ** 2.0 - y ** 2.0) - 10.0
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    # Make data.
    X = np.arange(0, 100, 0.25)
    Y = np.arange(0, 100, 0.25)
    X, Y = np.meshgrid(X, Y)
    Z1 = StandardReefFunction(X, Y)
    
    # Plot the surface.
    surf = ax.plot_surface(X, Y, Z1, cmap='gray',alpha=1)
    ax.set_aspect('equal', 'box')
    ax.grid(False)
    
    plt.show()

def InspectReefData2D():
    def StandardReefFunction(x, y):
        x = (x-0.5 *100)/15
        y = (y-0.5 *100)/15
        return (1.0 - x / 2.0 + x ** 5.0 + y ** 3.0) * np.exp(-x ** 2.0 - y ** 2.0) - 10.0
    
    fig, ax = plt.subplots()
    
    # Make data.
    X = np.arange(0, 100, 0.25)
    Y = np.arange(0, 100, 0.25)
    X, Y = np.meshgrid(X, Y)
    
    
    # reef levels
    plt.contourf(X, Y, StandardReefFunction(X, Y), 8, alpha=.75, cmap='gray')
    
    # contour lines
    ax.contourf(X, Y, StandardReefFunction(X, Y),levels = [-9.5, 10], colors = 'red', linestyles = 'solid',zorder=10)
    
    # some formatting
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    plt.title('2-D Cut at Depth z = -9.5')
    
    plt.show()
    
def InspectReefData2DComplete():
    UnknownRegions = { \
                                   0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                                   0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                                   0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                                   }
    
    def StandardReefFunction(x, y):
        x = (x-0.5 *100)/15
        y = (y-0.5 *100)/15
        return (1.0 - x / 2.0 + x ** 5.0 + y ** 3.0) * np.exp(-x ** 2.0 - y ** 2.0) - 10.0
    
    fig, ax = plt.subplots()
    
    # Make data.
    X = np.arange(0, 100, 0.25)
    Y = np.arange(0, 100, 0.25)
    X, Y = np.meshgrid(X, Y)
    
    
    # reef levels
    plt.contourf(X, Y, StandardReefFunction(X, Y), 8, alpha=.75, cmap='gray')
    
    # contour lines
    ax.contourf(X, Y, StandardReefFunction(X, Y),levels = [-9.5, 10], colors = 'red', linestyles = 'solid',zorder=10)
    
    # unknown regions
    from matplotlib import path as mpath
    import matplotlib.patches as patches

    for risk_key in UnknownRegions:
        unknown_region = patches.PathPatch(mpath.Path(UnknownRegions[risk_key]), facecolor='red', lw=0.0001, alpha = risk_key)
        ax.add_patch(unknown_region)
    
    # some formatting
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    plt.title('2-D Cut at Depth z = -9.5')
    
    plt.show()
