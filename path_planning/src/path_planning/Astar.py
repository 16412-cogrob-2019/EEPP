import numpy as np
from utils import *
#import rospy
#from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

# TODO: figure out how to do base speed. Currently hardcoded
auv_speed = 10.0

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
        if r == 1.0: # note to self: alternatively, r > 0.9 for tolerance
            return False
    return True

def Astar(map, start, goal, alpha):
    print("Running A* with:")
    print("Start:", start)
    print("Goal:", goal)
    print("Alpha:", alpha)
    #debug_pub = rospy.Publisher("/eepp/debug", Odometry, queue_size = 1000) # jsonified data

    # Create start and end node
    start_node = Node(None, start, map.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, goal, map.current_at(goal))
    goal_node.g = goal_node.h = goal_node.f = 0
    plt.plot(start_node.position[0], start_node.position[1], "g.")
    plt.plot(goal_node.position[0], goal_node.position[1], "r.")
    plt.pause(.0001)

    # Initialize both open and closed list
    open_list = NodePriorityQueue()
    closed_list = []

    # Add the start node
    start_node.V_AUV = np.array((0,0))
    open_list.put(start_node, start_node.f)

    start_node.risk = 0
    start_node.timeToChild = 0.001

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

        open_list.put(child, child.f)

    # Loop until you find the end
    i = 10
    while not open_list.empty():

        # Get the current node (node with smallest f value i.e. cost-to-go)
        current_node = open_list.pop()
        closed_list.append(current_node.position)

        step = map.res

        # Found the goal
        if dist(current_node, goal_node) <= step:
            (x,y) = goal_node.position
            path = [(x,y,np.linalg.norm(goal_node.speed))]
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
        for new_position in [(0.0, -step), (0.0, step), (-step, 0.0), (step, 0.0),
                             (-step, -step), (-step, step), (step, -step), (step, step)]: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # Make sure within range
            not_in_x_range = node_position[0] >= map.width + map.pos[0] or node_position[0] < map.pos[0]
            not_in_y_range = node_position[1] >= map.height + map.pos[1] or node_position[1] < map.pos[0]
            if not_in_x_range or not_in_y_range:
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
            if child.position in closed_list:
                discard = True

            # Check if child is already in the open list
            if open_list.in_queue(child.position):
                open_node = open_list.get_node(child.position)
                if child.f > open_node.f:
                        discard = True
                else:
                    open_list.delete(open_node.position)

            if discard:
                continue

            # if i % 10 == 0:
            #     cn = current_node.position
            #     nn = child.position
            #     plt.plot([nn[0]],[nn[1]],'.')
            #     plt.pause(0.0001)

            # i += 1

            update_vertex(current_node, child) # add extra argument True to apply any angle

            # msg = Odometry()
            # msg.header.frame_id="map"
            # msg.pose.pose.position.x = child.position[0]
            # msg.pose.pose.position.y = child.position[1]
            # msg.pose.pose.position.z = 0.0
            # debug_pub.publish(msg)

    return paths, costs

###############################################################################
# Testing
class MapObject(object):
    def risk_at(self, position):
        (x, y) = position
        if x >=0 and x<=20 and y>=0 and y<=20:
            return 1
        return 0
    def current_at(self, position):
        return (-1, 1)

map = MapObject()
map.res = 1
map.height = 100
map.width = 100
map.pos = [-50, -50]

paths, costs = Astar(map, (-30, -30), (40, 40), 1)
print("Test path:")
for pos in paths:
    print(pos)
print("Cost:", costs)

