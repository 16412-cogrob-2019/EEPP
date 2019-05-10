from path_planning.Astar import Astar
#import other algorithms

class PathTree():
    def __init__(self, path):
        self.path = path

    def update_tree(self, path):


def plan_path(map1, starts, goals, alg, alpha, prev_tree=None):
    if alg == "A*":
        # for start,goal in zip(starts, goals):
        path, cost = Astar(map1, start, goal, alpha)
    elif alg == "LPA*":

        path, cost = LPAStar(map1, start, goal, alpha, prev_tree)
        tree.update_tree(path)
    else:
        print("Path planning algorithm not recognized!")
        paths = None
        costs = None
    return paths, costs
