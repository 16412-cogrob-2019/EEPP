import numpy as np
from utils import *
import time
# import rospy
# from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

# TODO: figure out how to do base speed. Currently hardcoded
auv_speed = 3.0

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

def line_of_sight(node1, node2, map):
    (x1, y1) = node1.position
    (x2, y2) = node2.position

    dx = x2-x1
    dy = y2-y1
    steps = 1 + int(max(abs(dx),abs(dy))) # integer number of horizontal steps

    dx = dx/steps
    dy = dy/steps

    for i in range(steps):
        r = 0
        # also check surrounding squares
        for nx in [0]:#, 1, -1]:
            for ny in [0]:#, 1, -1]:
                x = x1 + dx*i + nx*4*map.res
                y = y1 + dy*i + ny*4*map.res
                r = max(r, map.risk_at((x, y)))
        # if r > 0.5:
        #     color = "r."
        # else:
        #     color = "g."
        # plt.plot(x,y,color)
        # plt.pause(0.0001)
        if r > 0.5: # == 1.0
            return False
    return True

def LPAStar(map1, start, goal, alpha, prev_tree, any_a=False, pri=False, c='r.'):
    # print("Running LPA* with:")
    # print("Start:", start)
    # print("Goal:", goal)
    # print("Alpha:", alpha)
    # debug_pub = rospy.Publisher("/eepp/debug", Odometry, queue_size = 1000) # jsonified data

    # Create start and end node
    start_node = Node(None, [None], start, map1.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, [None], goal, map1.current_at(goal))
    goal_node.g = goal_node.h = goal_node.f = 0

    plt.plot(start_node.position[0],start_node.position[1],'g.',ms=10)
    plt.plot(goal_node.position[0],goal_node.position[1],c,ms=10)
    plt.pause(0.0001)

    # Initialize both open and closed list
    open_list = NodePriorityQueue()
    closed_list = []

    # Add the start node
    start_node.V_AUV = np.array((0,0))
    open_list.put(goal_node, goal_node.f)

    goal_node.risk = map1.risk_at(goal_node.position)

    def update_node_values(parent):
        parent.risk = map1.risk_at(parent.position)

        time_to_child, parent.child[-1].V_AUV = cost_function(parent, parent.child[-1], auv_speed, alpha)
        parent.g = parent.child[-1].g + time_to_child

        # time_to_goal = heuristic(parent, start_node, auv_speed)
        time_to_goal = heuristic(start_node, parent, auv_speed)
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

        # Checks if we interesect the pre-existing tree anywhere
        tree_node = in_tree(current_node, prev_tree, step)
        # if tree_node:
        #     path = []
        #     path_node = prev_tree
        #     path_cost = prev_tree.g

        #     while path_node is not None:
        #         ()

        if dist(current_node, start_node) <= step/2.0 or tree_node != False:
            path = []
            path_cost = 0


            # (x,y) = goal_node.position
            # path = [(x,y,np.linalg.norm(goal_node.speed))]
            # If we intersect the tree, find the path from start to intersection
            if tree_node != False:
                # current_node.parent = tree_node
                path_node = tree_node
                # path_cost = current_node.g - tree_node.g
                while path_node is not None:
                    # print path_node
                    # print path_node
                    (x, y) = path_node.position
                    path.append((x, y, np.linalg.norm(path_node.speed)))
                    path_cost = path_node.g
                    path_node = path_node.parent
                    # temp.parent = path_node
                    # path_node = temp
                path_cost -= tree_node.g
                # print path_cost,"PN"
                path.reverse()
                tree_node.child.append(current_node)
                ret_node = prev_tree

            # Else, just go from current node (either at start or intersection) to goal
            else:
                if prev_tree is not None:
                    prev_tree.child.append(current_node)
                    current_node.parent = prev_tree
                    ret_node = prev_tree
                else:
                    ret_node = current_node
                # path_cost = current_node.g
                path = []

            # print current_node.g,"CN"
            path_cost += current_node.g
            current_node.g = path_cost
            path_node = current_node
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

            within_x_range = node_position[0] >= map1.width + map1.pos[0] or node_position[0] < map1.pos[0]
            within_y_range = node_position[1] >= map1.height + map1.pos[1] or node_position[1] < map1.pos[0]
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

            if i % 5 == 0 and pri:
                cn = current_node.position
                nn = new_node.position
                plt.plot(new_node.position[0],new_node.position[1],'b.')
                plt.pause(0.0001)

            i += 1

            update_vertex(current_node, new_node, any_a) # add extra argument True to apply any angle
            # msg = Odometry()
            # msg.header.frame_id="map"
            # msg.pose.pose.position.x = new_node.position[0]
            # msg.pose.pose.position.y = new_node.position[1]
            # msg.pose.pose.position.z = 0.0
            # debug_pub.publish(msg)
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

import current_types
import map_obj
# class Map:
#     """docstring for Map"""
#     def __init__(self, map_msg, add_blur=True):
#         self.array = self.update_grid(map_msg.data)
#         self.height = map_msg.info.height #Ngridpoints
#         self.width = map_msg.info.width #Ngridpoints
#         self.res = map_msg.info.resolution #m/cell
#         self.origin = map_msg.info.origin
#         self.pos = [self.origin.position.x, self.origin.position.y]
#         self.grid = np.asarray(self.array, dtype=np.int8).reshape(self.height, self.width)
#         self.risk = self.get_risk_field(self.grid, add_blur)
#         self.current = current_types.Current("Sine Waves Horiz", 1, np.zeros_like(self.grid,dtype="float"))

class Position:
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Origin:
    def __init__(self,position):
        self.position = position

class Info:
    def __init__(self,height,width,origin,resolution):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

class MSG:
    def __init__(self,info,data):
        self.info = info
        self.data = data


map1 = MapObject()
map1.res = 1
map1.height = 100
map1.width = 100
map1.pos = [-50, -50]

pos = Position(0,0)
o = Origin(pos)
h = 100
w = 100
r = 1
I = Info(h,w,o,r)
D = np.zeros(100*100)
map_msg = MSG(I,D)

map_msg = np.loadtxt("map_dump_grid_raw.txt")
new_msg = np.copy(map_msg)
for i,value in enumerate(map_msg):
    for j,v in enumerate(value):
        if v < 0:
            neighbors = False
            for k in [(0,1),(1,0),(1,1),(-1,0),(0,-1),(-1,-1),(-1,1),(1,-1)]:
                if k[0]+i >= 0 and k[0]+i < len(map_msg) and k[1]+j >= 0 and k[1]+j < len(value):
                    neigh = map_msg[k[0]+i,k[1]+j]
                    # print neigh
                    if neigh > 0:
                        # plt.plot(k[0]+i,k[1]+j,'.')
                        # plt.pause(0.0001)
                        neighbors = True
            if neighbors:
                new_msg[i,j] = 1
            else:
                new_msg[i,j] = .2
        else:
            new_msg[i,j] = map_msg[i,j]/100.


map_msg = new_msg
h,w = map_msg.shape

map2 = map_obj.MapObj(map_msg, False)
# map2.add_obstacle()
plt.imshow(map2.grid,cmap="binary")
plt.pause(0.0001)
# plt.show()

start = (200, 160)
end = (230, 200)
# end=(219,200)


pri = False
any_a = False
plt.streamplot(np.arange(h),np.arange(w),map2.current.current_x,map2.current.current_y,density=1)
# plt.quiver(np.arange(h),np.arange(w),map2.current.current_x,map2.current.current_y)#,scale=.5,headwidth=3,color="Blue")
start_time = time.time()
ans = LPAStar(map2, start, end, 1, None, any_a, pri)
# plt.clf()
# plt.imshow(map1)
if ans is not None:
    path, cost, tree = ans
    print "Total time 1st run: ", time.time() - start_time, "RED DOT"
    print "Path Cost", cost
    plt.plot(start[0], start[1], 'g.')
    plt.plot(end[0], end[1], 'r.')
    for p in range(1, len(path)):
        plt.plot([path[p-1][0],path[p][0]],[path[p-1][1],path[p][1]],'k-')
        plt.pause(.0001)
# plt.show()
pri = False
ends = [(230,160),(200,200),(217,200)]
colors = ["BLUE DOT", "MAGENTA DOT", "CYAN DOT"]
c = ["b.","m.","c."]
for i in range(2,5):
    start_time = time.time()
    end = tuple((np.random.rand(2)*90).astype("int"))

    ans = LPAStar(map2, start, ends[i-2], 1, tree, any_a, pri,c[i-2])
    if ans is not None:
        path, cost, tree = ans
        th = "nd" if i == 2 else "rd" if i == 3 else "th"
        print "Total time %d"%i,th," run: ", time.time() - start_time, colors[i-2]
        print "Path Cost", cost
        for p in range(1, len(path)):
            plt.plot([path[p-1][0],path[p][0]],[path[p-1][1],path[p][1]],'k-')
            plt.pause(.0001)
    plt.pause(1)
plt.show()

# print("Test path:")
# for pos in path:
#     print(pos)
# print("Cost:", cost)

