from path_planning.Astar import Astar
from path_planning.LPAStar import LPAStar
#import other algorithms

def plan_path(map1, start, goal, alg, alpha, prev_tree=None):
    if alg == "A*":
        # for start,goal in zip(starts, goals):
        path, cost = Astar(map1, start, goal, alpha)
        tree = None
    elif alg == "LPA*":
        path, cost, tree = LPAStar(map1, start, goal, alpha, prev_tree)
    else:
        print("Path planning algorithm not recognized!")
        path = None
        cost = None
        tree = None
    return path, cost, tree
