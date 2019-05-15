import numpy as np
import matplotlib.pyplot as plt
from utils import *
class Node:
    """A node class for path planning"""
    def __init__(self, parent=None, position=None, U= None, V=None, current_cost=np.inf):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
        self.current = currentAt(self, U, V)
        self.cost = current_cost
        self.transition_cost = None
        self.V_AUV = None
        self.V_AUVs = None
        self.risk = None
    def __str__(self):
        return str(self.position)# + "," + str(self.parent) + "," + str(self.cost)
    def __repr__(self):
        return str(self.position)# + "," + str(self.parent) + "," + str(self.cost) + "," + str(self.transition_cost)
    def __eq__(self, other):
        return self.position == other.position
    def update_transitions(self, edges):
        self.transition_cost = edges
    def update_current_cost(self, new_cost):
        self.cost = new_cost
    def update_vauvs(self, vauvs):
        self.V_AUVs = vauvs
        
"""
Compute current at the given node, using matrices U and V of X,Y velocity components respectively
"""
def currentAt(node, U, V):
    if U is None or V is None:
        return None
    return np.array((U[node.position[1], node.position[0]], V[node.position[1], node.position[0]]))

"""
The main A* algorithm.
Input: an AUV object and an Environment object
Output: the computed path as a list of Nodes, in the order they are visited
        each node has .VUAV attribute, which is the AUV velocity vector at that node
"""
def astar(auv, env, cost, heuristic, alpha, vis):
    
    start = auv.origin
    end = auv.goal
    X = env.X
    Y = env.Y
    U, V = env.CurrentField_x, env.CurrentField_y
    # Create start and end node
    start_node = Node(None, start, U, V)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end, U, V)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    start_node.V_AUV = np.array((0,0))
    open_list.append(start_node)

    start_node.risk = 0
    start_node.timeToChild = 0.001
    # Loop until you find the end
    Qmax = 0
    while len(open_list) > 0:
        
        ### space complexity measurement
        Qmax = max(Qmax,len(open_list))
        ###
        
        # Get the current node (node with smallest f value i.e. cost-to-go)
        current_node = open_list[0]     
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        
        # Found the goal
        if dist(current_node, end_node) < 0.01:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = current.parent
            
            # make sure we have a V_AUV for the first way point (added by Michael Schmid on 4/27/2019)
            # Return reversed path
            path = path[::-1]
            _, path[0].V_AUV = cost(path[0], path[1], auv.speed, alpha) 
            # change to new path interface
            path = [ {'position': pt.position, 'V_AUV': pt.V_AUV} for pt in path ]
            print("MaxQueueLength: ",Qmax)
            return path 

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2])
            # Make sure within range
            if node_position[0] > env.Dimension_x or node_position[0] < 0 or node_position[1] > env.Dimension_y or node_position[1] < 0:
                continue
            # Create new node and add to the children list
            new_node = Node(current_node, node_position, U, V)
            children.append(new_node)

        # Now loop through children
        for child in children:
            # Create the f, g, and h values
            child.risk = env.actualRisk(child.position[0], child.position[1], child.position[2])
            if cost.__name__ == "cost_no_current_no_risk":
                child.timeToChild, child.V_AUV = cost(current_node, child, auv.speed)
            else: 
                child.timeToChild, child.V_AUV = cost(current_node, child, auv.speed, alpha)
            child.g = current_node.g + child.timeToChild

            timeToGoal = heuristic(child, end_node, auv.speed)
            child.h = timeToGoal
            child.f = child.g + child.h
            
            # if we find the child on either the open or closed list **with a better cost** we discard the child
            discard = False
            
            # Check if child is already in the open list
            foundInOpen = False
            for i, open_node in enumerate(open_list):
                if child == open_node:
                    foundInOpen = True
                    openIdx = i
                    if child.f > open_node.f:
                        discard = True
                        
            
            #Check if child is already in the closed list
            foundInClosed = False            
            for i, closed_node in enumerate(closed_list):
                if child == closed_node:
                    foundInClosed = True
                    closedIdx = i
                    if child.f > closed_node.f:
                        discard = True
                        
            if discard:
                continue
            
            #remove old occurances of the node, and add the child to the open list
            if foundInOpen:
                open_list.pop(openIdx)
            if foundInClosed:
                closed_list.pop(closedIdx)
            open_list.append(child)
            