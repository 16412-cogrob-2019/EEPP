import numpy as np
from utils import *
import time
#import rospy
#from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

# TODO: figure out how to do base speed. Currently hardcoded
auv_speed = 10.0

class Node:
    def __init__(self, parent=None, child=[None], position=None, current=(0, 0)):
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
    dx = (x2 - x1)
    dy = (y2 - y1)
    steps = 1 + int(max(abs(dx), abs(dy))) # integer number of horizontal steps

    dx = dx/steps
    dy = dy/steps

    for i in range(steps):
        r = map1.risk_at((x1 + dx*i, y1 + dy*i))
        if r == 1.0: # note to self: alternatively, r > 0.9 for tolerance
            return False
    return True

def LPAStar(map1, start, goal, alpha, prev_tree, any_a=True, pri=False):
    print("Running LPA* with:")
    print("Start:", start)
    print("Goal:", goal)
    print("Alpha:", alpha)
    #debug_pub = rospy.Publisher("/eepp/debug", Odometry, queue_size = 1000) # jsonified data

    # Create start and end node
    start_node = Node(None, [None], start, map1.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, [None], goal, map1.current_at(goal))
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
    if goal_node.risk > 0:
        print "This is an obstacle!"
        return None
    # goal_node.timeToChild = 0.001

    def update_node_values(parent):
        parent.risk = map1.risk_at(parent.position)

        time_to_child, parent.child[-1].V_AUV = cost_function(parent, parent.child[-1], auv_speed, alpha)
        parent.g = parent.child[-1].g + time_to_child

        time_to_goal = heuristic(parent, start_node, auv_speed)
        parent.h = time_to_goal
        parent.f = parent.g + parent.h
        return parent

    def update_vertex(current_node, parent, use_any_angle=False):
        if use_any_angle:
            child = current_node.child[-1]
            if child and line_of_sight(parent, child, map1):
                # New f, g, and h values
                # child.parent = parent
                parent.child.append(child)
                parent = update_node_values(parent)
            else: pass
        else: pass # if just using regular A*, just add directly to open list

        open_list.put(parent, parent.f)
    i = 0
    # Loop until you find the end
    while not open_list.empty():

        # Get the current node (node with smallest f value i.e. cost-to-go)
        current_node = open_list.pop()
        closed_list.append(current_node.position)

        step = map1.res

        # Found the goal

        tree_node = in_tree(current_node, prev_tree, step)
        # if tree_node:
        #     path = []
        #     path_node = prev_tree
        #     path_cost = prev_tree.g

        #     while path_node is not None:
        #         ()

        if dist(current_node, start_node) <= step/2.0 or tree_node != False:
            path = []



            # (x,y) = goal_node.position
            # path = [(x,y,np.linalg.norm(goal_node.speed))]

            if tree_node != False:
                path_node = tree_node

                while path_node is not None:
                    # print path_node
                    # print path_node
                    (x, y) = path_node.position
                    path.append((x, y, np.linalg.norm(path_node.speed)))
                    path_node = path_node.parent
                    # temp.parent = path_node
                    # path_node = temp
                path.reverse()
                tree_node.child.append(current_node)
                ret_node = prev_tree

            else:
                if prev_tree is not None:
                    prev_tree.child.append(current_node)
                    current_node.parent = prev_tree
                    ret_node = prev_tree
                else:
                    ret_node = current_node
                path = []

            path_node = current_node
            path_cost = current_node.g
            while path_node is not None:
                # print path_node
                # print path_node
                (x, y) = path_node.position
                path.append((x, y, np.linalg.norm(path_node.speed)))
                temp = path_node.child[-1]
                if temp is not None:
                    temp.parent = path_node
                path_node = temp

            # (x,y) = start_node.position
            # path.append((x,y,np.linalg.norm(start_node.speed)))

            # Return reversed path in (x, y, speed), as well as total path cost

            return path, path_cost, ret_node


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
            new_node = Node(None, [current_node], node_position, map1.current_at(node_position))
            # children.append(new_node)

        # # Now loop through children
        # for child in children:

            # Create the f, g, and h values
            new_node = update_node_values(new_node)
            # current_node.parent = new_node
            # print current_node, new_node
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

            if i % 1 == 0 and pri:
                cn = current_node.position
                nn = new_node.position
                plt.plot([nn[0]],[nn[1]],'.')
                plt.pause(0.0001)

            i += 1

            update_vertex(current_node, new_node, any_a) # add extra argument True to apply any angle
            #msg = Odometry()
            #msg.header.frame_id="map1"
            #msg.pose.pose.position.x = child.position[0]
            #msg.pose.pose.position.y = child.position[1]
            #msg.pose.pose.position.z = 0.0
            #debug_pub.publish(msg)
    print "No path found"
    return None

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

start = (-30, -30)
end = (40, 40)

start_time = time.time()
pri = False
any_a = True
ans = LPAStar(map1, start, end, 1, None, any_a, pri)
plt.clf()
if ans is not None:
    path, cost, tree = ans
    print "Total time 1st run: ", time.time() - start_time
    plt.plot(start[0], start[1], 'g.')
    plt.plot(end[0], end[1], 'r.')
    for p in range(1, len(path)):
        plt.plot([path[p-1][0],path[p][0]],[path[p-1][1],path[p][1]],'b-')
        plt.pause(.0001)
# plt.show()
pri = True
for i in range(2,11):
    start_time = time.time()
    ans = LPAStar(map1, start, tuple((np.random.rand(2)*70-30).astype("int")), 1, tree, any_a, pri)
    if ans is not None:
        path, cost, tree = ans
        th = "nd" if i == 2 else "rd" if i == 3 else "th"
        print "Total time %d"%i,th," run: ", time.time() - start_time
        for p in range(1, len(path)):
            plt.plot([path[p-1][0],path[p][0]],[path[p-1][1],path[p][1]],'b-')
            plt.pause(.0001)
    plt.pause(10)
plt.show()

# print("Test path:")
# for pos in path:
#     print(pos)
# print("Cost:", cost)