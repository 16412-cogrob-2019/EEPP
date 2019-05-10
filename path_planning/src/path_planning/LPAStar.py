import numpy as np
from utils import *
#import rospy
#from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

# TODO: figure out how to do base speed. Currently hardcoded
auv_speed = 3.0

class Node:
    def __init__(self, parent=None, child=None, position=None, current=(0, 0)):
        self.parent = parent
        self.child = child
        self.position = position
        self.current = current
        self.speed = 0
        self.g = 0
        self.h = 0
        self.f = 0
        self.risk = None
    def __str__(self):
        return str(self.position)

def line_of_sight(node1, node2, map1):
    (x1, y1) = node1.position
    (x2, y2) = node2.position

    # TODO: steps can be parameterized maybe
    steps = 1 + int(abs(x2 - x1)) # integer number of horizontal steps

    dx = (x2 - x1)/steps
    dy = (y2 - y1)/steps

    for i in range(steps):
        r = map1.risk_at((x1 + dx*i, y1 + dy*i))
        if r == 1.0: # note to self: alternatively, r > 0.9 for tolerance
            return False
    return True

def LPAStar(map1, start, goal, alpha, prev_tree):
    print("Running LPA* with:")
    print("Start:", start)
    print("Goal:", goal)
    print("Alpha:", alpha)
    #debug_pub = rospy.Publisher("/eepp/debug", Odometry, queue_size = 1000) # jsonified data

    # Create start and end node
    start_node = Node(None, None, start, map1.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, None, goal, map1.current_at(goal))
    goal_node.g = goal_node.h = goal_node.f = 0
    plt.plot(start_node.position[0], start_node.position[1], "g.")
    plt.plot(goal_node.position[0], goal_node.position[1], "r.")
    plt.pause(.0001)

    # Initialize both open and closed list
    open_list = NodePriorityQueue()
    closed_list = []

    # Add the start node
    start_node.V_AUV = np.array((0,0))
    open_list.put(goal_node, goal_node.f)

    goal_node.risk = map1.risk_at(goal_node.position)
    # goal_node.timeToChild = 0.001

    def update_node_values(parent):
        parent.risk = map1.risk_at(parent.position)

        time_to_child, parent.child.V_AUV = cost_function(parent, parent.child, auv_speed, alpha)
        parent.g = parent.child.g + time_to_child

        time_to_goal = heuristic(parent, start_node, auv_speed)
        parent.h = time_to_goal
        parent.f = parent.g + parent.h
        return parent

    def update_vertex(current_node, parent, use_any_angle=False):
        if use_any_angle:
            child = current_node.child
            if child and line_of_sight(parent, child, map1):
                # New f, g, and h values
                child.parent = parent
                parent.child = child
                parent = update_node_values(parent)
            else: pass
        else: pass # if just using regular A*, just add directly to open list

        open_list.put(parent, parent.f)

    # Loop until you find the end
    while not open_list.empty():

        # Get the current node (node with smallest f value i.e. cost-to-go)
        current_node = open_list.pop()
        closed_list.append(current_node.position)

        step = map1.res

        # Found the goal

        tree_node = in_tree(current_node, prev_tree, step)
        if dist(current_node, start_node) <= step/2.0 or tree_node:
            if tree_node:
                current_node.parent = tree_node
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
        # children = []
        for new_position in [(0.0, -step), (0.0, step), (-step, 0.0), (step, 0.0),
                             (-step, -step), (-step, step), (step, -step), (step, step)]: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            # Make sure within range
            within_x_range = node_position[0] > map1.width + map1.pos[0] or node_position[0] < map1.pos[0]
            within_y_range = node_position[1] > map1.height + map1.pos[1] or node_position[1] < map1.pos[0]
            if within_x_range or within_y_range:
                continue
            # Create new node and add to the children list
            new_node = Node(None, current_node, node_position, map1.current_at(node_position))
            # children.append(new_node)
            cn = current_node.position
            nn = new_node.position
            plt.plot([nn[0]],[nn[1]],'.')
            plt.pause(0.0001)
        # # Now loop through children
        # for child in children:

            # Create the f, g, and h values
            new_node = update_node_values(new_node)
            current_node.parent = new_node

            # if we find the child on either the open or closed list **with a better cost** we discard the child
            discard = False

            #Check if child is already in the closed list
            if new_node.position in closed_list:
                discard = True

            # Check if child is already in the open list
            if open_list.in_queue(new_node.position):
                open_node = open_list.get_node(new_node.position)
                if new_node.f > open_node.f:
                        discard = True
                else:
                    open_list.delete(open_node.position)

            if discard:
                continue

            update_vertex(current_node, new_node) # add extra argument True to apply any angle
            #msg = Odometry()
            #msg.header.frame_id="map1"
            #msg.pose.pose.position.x = child.position[0]
            #msg.pose.pose.position.y = child.position[1]
            #msg.pose.pose.position.z = 0.0
            #debug_pub.publish(msg)
    return paths, costs

###############################################################################
# Testing

class MapObject(object):
    def risk_at(self, position):
        (x, y) = position
        # if x >=0 and x<=20 and y>=0 and y<=20:
        #     return 1
        return 0
    def current_at(self, position):
        return (-1, 1)

map1 = MapObject()
map1.res = 1
map1.height = 100
map1.width = 100
map1.pos = [-50, -50]

paths, costs = LPAStar(map1, (-30, -30), (40, 40), 1, None)
print("Test path:")
for pos in paths:
    print(pos)
print("Cost:", costs)

