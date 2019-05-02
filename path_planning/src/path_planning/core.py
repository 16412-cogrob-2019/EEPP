from path_planning.Astar import Astar
#import other algorithms
def plan_path(map, start, goal, alg, alpha):
    if alg == "A*":
        paths, costs = Astar(map, start, goal, alpha)
    else:
        print("Algorithm not recognized!")
        paths = None
        costs = None

    return paths, costs
