import numpy as np
import matplotlib.pyplot as plt
from utils import *
from GraphUtils import Node
import time

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
            
#             # performance measurements
#             t1 = time.time()
#             print("Plan-# ... in ... ", t1-t0, " seconds.")
            print("A*: ","MaxQueueLength: ",Qmax)
            
            # make sure we have a V_AUV for the first way point (added by Michael Schmid on 4/27/2019)
            # Return reversed path
            path = path[::-1]
            _, path[0].V_AUV = cost(path[0], path[1], auv.speed, alpha) 
            # change to new path interface
            #path = [ {'position': pt.position, 'V_AUV': pt.V_AUV} for pt in path ]
            return path,current_node.f 

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
            