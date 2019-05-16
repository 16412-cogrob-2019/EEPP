import numpy as np
from utils import *
# import rospy
# from nav_msgs.msg import Odometry

auv_speed = 3.0

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

    dx = x2-x1
    dy = y2-y1
    steps = 1 + int(max(abs(dx),abs(dy))) # integer number of horizontal steps

    dx = dx/steps
    dy = dy/steps

    for i in range(steps):
        r = 0
        # also check surrounding squares
        for nx in [0, 1, -1]:
            for ny in [0, 1, -1]:
                x = x1 + dx*i + nx*4*map.res
                y = y1 + dy*i + ny*4*map.res
                r = max(r, map.risk_at((x, y)))
        if r > 0.5: # == 1.0
            return False
    return True

def Astar(map, start, goal, alpha, any_a=False, pri=False, c='r.'):
    # print("Running A* with:")
    # print("Start:", start)
    # print("Goal:", goal)
    # print("Alpha:", alpha)
    # debug_pub = rospy.Publisher("/eepp/debug", Odometry, queue_size = 1000) # jsonified data

    # Create start and end node
    start_node = Node(None, start, map.current_at(start))
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, goal, map.current_at(goal))
    goal_node.g = goal_node.h = goal_node.f = 0

    plt.plot(start_node.position[0],start_node.position[1],'g.',ms=10)
    plt.plot(goal_node.position[0],goal_node.position[1],c,ms=10)
    plt.pause(0.0001)

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
    i = 0
    while not open_list.empty():

        # Get the current node (node with smallest f value i.e. cost-to-go)
        current_node = open_list.pop()
        closed_list.append(current_node.position)

        step = map.res
        # step = 4*map.res

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

            if i % 5 == 0 and pri:
                cn = current_node.position
                nn = new_node.position
                plt.plot(new_node.position[0],new_node.position[1],'b.')
                plt.pause(0.0001)

            i += 1

            i += 1

            update_vertex(current_node, child, any_a) # add extra argument True to apply any angle

            # msg = Odometry()
            # msg.header.frame_id="map"
            # msg.pose.pose.position.x = child.position[0]
            # msg.pose.pose.position.y = child.position[1]
            # msg.pose.pose.position.z = 0.0
            # debug_pub.publish(msg)

    return paths, costs


import current_types
import map_obj
import matplotlib.pyplot as plt
import time
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


pri = False
any_a = False
plt.streamplot(np.arange(h),np.arange(w),map2.current.current_x,map2.current.current_y,density=2)
# plt.quiver(np.arange(h),np.arange(w),map2.current.current_x,map2.current.current_y)#,scale=.5,headwidth=3,color="Blue")
start_time = time.time()
ans = Astar(map2, start, end, 1, any_a, pri)
# plt.clf()
# plt.imshow(map1)
if ans is not None:
    path, cost = ans
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

    ans = Astar(map2, start, ends[i-2], 1, any_a, pri,c[i-2])
    if ans is not None:
        path, cost = ans
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

