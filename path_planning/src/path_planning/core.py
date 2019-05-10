from path_planning.Astar import Astar
#import other algorithms

class PathTree():
    def __init__(self, path):
        self.path = path


def plan_path(map1, start, goal, alg, alpha, prev_tree=None):
    if alg == "A*":
        # for start,goal in zip(starts, goals):
        path, cost = Astar(map1, start, goal, alpha)
    elif alg == "LPA*":

        path, cost = LPAStar(map1, start, goal, alpha, prev_tree)
        tree.update_tree(path)
    else:
        print("Path planning algorithm not recognized!")
        path = None
        cost = None
    return path, cost
