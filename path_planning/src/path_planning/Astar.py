import numpy as np
from utils import *

# TODO: figure out how to do base speed. Currently hardcoded
auv_speed = 10

class Node:
    def __init__(self, parent=None, position=None, current=(0, 0)):
        self.parent = parent
        self.position = position
        self.current = current
        self.speed = 0
        self.g = 0
        self.h = 0
        self.f = 0
        self.risk = None
    def __str__(self):
        return str(self.position)
    
def line_of_sight(node1, node2, map):
    (x1, y1) = node1.position
    (x2, y2) = node2.position
    
    # TODO: steps can be parameterized maybe
    steps = 1 + int(abs(x2 - x1)) # integer number of horizontal steps
    
    dx = (x2 - x1)/steps
    dy = (y2 - y1)/steps
    
    for i in range(steps):
        r = map.risk_at((x1 + dx*i, y1 + dy*i))
        if r == 1: # note to self: alternatively, r > 0.9 for tolerance
            return False
    return True

def Astar(map, start, goal, alpha):
    print("Running A* with:")
    print("Start:", start)
    print("Goal:", goal)
    print("Alpha:", alpha)
    
    # Create start and end node
    start_node = Node(None, start, map.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, goal, map.current_at(goal))
    goal_node.g = goal_node.h = goal_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    start_node.V_AUV = np.array((0,0))
    open_list.append(start_node)

    start_node.risk = 0
    start_node.timeToChild = 0.001
    
    # Step for grid size
    step = map.res
    
    def update_node_values(child):
        child.risk = map.risk_at(child.position)
        
        child.timeToChild, child.speed = cost_function(child.parent, child, auv_speed, alpha)
        child.g = child.parent.g + child.timeToChild

        timeToGoal = heuristic(child, goal_node, auv_speed)
        child.h = timeToGoal
        child.f = child.g + child.h
        return child
    
    def update_vertex(current_node, child, use_any_angle=False):
        if use_any_angle:
            parent = current_node.parent
            if parent and line_of_sight(parent, child, map):
                # New f, g, and h values
                child.parent = parent
                child = update_node_values(child)
            else: pass
        else: pass # if just using regular A*, just add directly to open list 
        
        open_list.append(child)
    
    # Loop until you find the end
    while open_list:
        
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
        if dist(current_node, goal_node) <= step:
            path = []
            path_node = current_node
            path_cost = current_node.g
            
            while path_node is not None:
                (x, y) = path_node.position
                path.append((x, y, np.linalg.norm(path_node.speed)))
                path_node = path_node.parent
            
            # Return reversed path in (x, y, speed), as well as total path cost
            return path[::-1], path_cost

        # Generate children
        children = []
        for new_position in [(0, -step), (0, step), (-step, 0), (step, 0), 
                             (-step, -step), (-step, step), (step, -step), (step, step)]: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # Make sure within range
            within_x_range = node_position[0] > map.width + map.pos[0] or node_position[0] < map.pos[0]
            within_y_range = node_position[1] > map.height + map.pos[1] or node_position[1] < map.pos[0]
            if within_x_range or within_y_range:
                continue
            # Create new node and add to the children list
            new_node = Node(current_node, node_position, map.current_at(node_position))
            children.append(new_node)

        # Now loop through children
        for child in children:
            
            # Create the f, g, and h values
            child = update_node_values(child)
            
            # if we find the child on either the open or closed list **with a better cost** we discard the child
            discard = False
            
            #Check if child is already in the closed list
            foundInClosed = False            
            for i, closed_node in enumerate(closed_list):
                if child.position == closed_node.position:
                    foundInClosed = True
                    closedIdx = i
                    discard = True
                        
            # Check if child is already in the open list
            foundInOpen = False
            for i, open_node in enumerate(open_list):
                if child.position == open_node.position:
                    foundInOpen = True
                    openIdx = i
                    if child.f > open_node.f:
                        discard = True
                        
            if discard:
                continue
            
            #remove old occurances of the node, and add the child to the open list
            if foundInOpen:
                open_list.pop(openIdx)
            if foundInClosed:
                closed_list.pop(closedIdx)
            
            update_vertex(current_node, child) # add extra argument True to apply any angle
    
    return paths, costs

###############################################################################
# Testing   
'''
class MapObject(object):
    def risk_at(self, position):
        (x, y) = position
        if x >=1 and x<=10 and y>=1 and y<=10:
            return 1
        return 0
    def current_at(self, position):
        return (-4, -3)

map = MapObject()
map.res = 1
map.height = 10
map.width = 10
map.pos = [-2, -2]

paths, costs = Astar(map, (0, 0), (3, 4), 0.5)
print("Test path:")
for pos in paths:
    print(pos)
print("Cost:", costs)
'''
