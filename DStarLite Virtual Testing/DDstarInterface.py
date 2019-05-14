from Dutility import grid_from_string,draw_grid
from Ddstarlite import DStarLite
import numpy as np
from WWWDSTAR import explore

GRAPH, START, END = grid_from_string("""
    ..........
    ...######.
    .......A#.
    ...######.
    ...#....#.
    ...#....#.
    ........#.
    ........#.
    ........#Z
    ........#.
    """)

# create Dstarinstance
dstar = DStarLite(GRAPH, START, END)

###Test
print(GRAPH.width)
print(GRAPH.height)
print(GRAPH.walls)
###Test

# call Dstar 
path = [p for p, o, w in dstar.move_to_goal()]
#path = [p for p, o, w in explore(dstar)]

# visualization
print("The graph (A=Start, Z=Goal)")
draw_grid(GRAPH, width=3, start=START, goal=END)
print("\n\nPath taken (@ symbols)")
draw_grid(GRAPH, width=3, path=path)

    
    