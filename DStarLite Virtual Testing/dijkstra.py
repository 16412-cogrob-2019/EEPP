from auv import *
from environment import *
from Astar import *
import matplotlib.animation as animation

adjacent = [(0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0), (-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0)]


def dijkstra_planning(cost_map, start, goal, vis):

    nstart = Node(None, start, None, None, 0)
    ngoal = Node(None, goal)
    nstart.V_AUV = np.array([0,0])
    ngoal.V_AUV = np.array([0,0])
    nstart.V_AUVs = np.array([0,0])
    ngoal.V_AUVs = np.array([0,0])

    openset, closedset = dict(), dict()
    closedlist = []
    for xind,col in enumerate(cost_map):
        for yind,val in enumerate(col):
            # openset[(xind, yind, start[2])] = Node(None, (xind, yind, start[2]))
            openset[(xind, yind, start[2])] = val

    openset[nstart.position] = nstart
    # current = nstart

    while True:
        c_id = min(openset, key=lambda x: openset[x].cost)
        current = openset[c_id]

        # # show graph
        # if vis is not None and len(closedset.keys()) % 10 == 0:
        #     vis.ax.plot(current.position[0], current.position[1], "xg")
        #     plt.pause(0.0001)
        closedlist.append(c_id)

        if c_id == goal:
            print('Goal found!')
            ngoal.parent = current.parent
            ngoal.cost = current.cost

            if vis is not None:
                print('Preparing visualization...')
                # Writer = animation.writers['ffmpeg']
                # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
                skip = 15
                scat = vis.ax.scatter([], [], s=10, facecolor=None, alpha=0.5)
                closedlist= np.array(closedlist)
                def init():
                    scat.set_offsets([])
                    return scat,

                def animate(i):
                    scat.set_offsets(closedlist[:i,:2])
                    return scat,

                vis.searchAnimation = animation.FuncAnimation(vis.fig, animate, init_func=init, frames= range(len(closedlist)+1)[::skip], interval=100, blit=True, repeat=False)
                # plt.pause(0.001)
                # anim.save('djikstra.mp4', writer=writer)
#                 plt.show()
                print('Complete!')
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search Dgrid based on motion model
        for i, cost in enumerate(cost_map[c_id[0]][c_id[1]].transition_cost):
            neighbor = tuple(map(sum,zip(c_id, adjacent[i])))
            # cost is none if neighbor is out of bounds
            if cost is not None and neighbor not in closedset:

                new_cost = current.cost + cost
                # node = Node(c_id, neighbor, None, None, new_cost)
                if neighbor in openset:
                    if openset[neighbor].cost > new_cost:
                        neighbor_node = cost_map[neighbor[0]][neighbor[1]]
                        neighbor_node.parent = c_id
                        neighbor_node.position = neighbor
                        neighbor_node.update_current_cost(new_cost)
                        neighbor_node.V_AUV = neighbor_node.V_AUVs[i]
                        openset[neighbor] = neighbor_node
                        # print(neighbor_node.V_AUV)
                        # openset[neighbor] = node
                        # openset[n_id].pind = c_id
                else:
                    # neighbor_node.update_current_cost(new_cost)
                    neighbor_node = cost_map[neighbor[0]][neighbor[1]]
                    neighbor_node.parent = c_id
                    neighbor_node.position = neighbor
                    neighbor_node.update_current_cost(new_cost)
                    neighbor_node.V_AUV = neighbor_node.V_AUVs[i]
                    openset[neighbor] = neighbor_node
                    # openset[neighbor] = node

    # generate final course
    # rx, ry, rz = ngoal.position
    rx = [ngoal.position[0]]
    ry = [ngoal.position[1]]
    nodes = [ngoal]
    parent = ngoal.parent
    while parent is not None:
        n = closedset[parent]
        rx.append(n.position[0])
        ry.append(n.position[1])
        nodes.append(n)
        parent = n.parent

    # return rx[::-1], ry[::-1], n[::-1]
#     if vis is not None:
#         vis.ax.plot(rx[::-1],ry[::-1],color='k',linewidth=0.5)
    return nodes[::-1]


def generate_cost_map(U, V, auv_speed, alpha, env, auv, costfunction):
    z = auv.origin[2]
    cost_map = np.ones_like(U).tolist()
    for x_ind,Ucol in enumerate(U):
        for y_ind,val in enumerate(Ucol):
            node1 = Node(None,[x_ind,y_ind,z],U,V)
            edges = []
            vauvs = []
            for pos in adjacent:
                x2,y2 = pos[0] + x_ind, pos[1] + y_ind
                if x2 >= 0 and x2 < env.discretization_x and y2 >= 0 and y2 < env.discretization_y:
                    node2 = Node(None,[x2,y2,z],U,V)
                    node2.risk = env.actualRisk(node2.position[0],node2.position[1],node2.position[2])
                    
                    if costfunction.__name__ == "cost_no_current_no_risk":
                        cost, V_AUV = costfunction(node1, node2, auv_speed)
                    else:
                        cost, V_AUV = costfunction(node1, node2, auv_speed, alpha)
                    
                    #cost, V_AUV = costfunction(node1, node2, auv_speed)#, alpha)
                    vauvs.append(V_AUV)
                    edges.append(cost)
                else:
                    vauvs.append(None)
                    edges.append(None)
            # print(edges)
            node1.update_transitions(edges)
            node1.update_vauvs(vauvs)
            # print (x_ind, y_ind)
            # print(node1)
            # print(cost_map)
            cost_map[x_ind][y_ind] = node1
    

    return cost_map


def global_planner(auv, env, vis, cost, alpha):
    np.set_printoptions(threshold = np.nan)
    start = auv.origin
    end = auv.goal
    if start == (None,None,None):
        start = (0,50)
    if end == (None,None,None):
        end = (100,50)
    X = env.X
    Y = env.Y
    U, V = env.CurrentField_x, env.CurrentField_y
    risk = env.RiskField
    auv_speed = auv.speed
    print('Generated Cost map...')
    cost_map = generate_cost_map(U,V,auv_speed, alpha, env, auv, cost)
    print('Completed!')
    print('Searching for path to goal...')
    nodes = dijkstra_planning(cost_map, start, end, vis)
    # rx,ry=[],[]
    # for node in nodes:
    #     print(node)
    #     rx.append(node.position[0]/100)
    #     ry.append(node.position[1]/100)
    # plt.plot(rx, ry, "-r")
    # plt.pause(0.001)
    # plt.show()
    return nodes

if __name__ == '__main__':
    mission = Mission(1,1,101)
    auv = AUV()
    env = ENVIRONMENT(mission)
    global_planner(auv, env)