# utility functions for dealing with square grids
import Dgrid as grid
from Dpriority_queue import PriorityQueue

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, start, goal):
    """Reconstruct a shortest path from a dictionary of back-pointers"""
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.append(start)  # optional
    path.reverse()  # optional
    return path

######################################################################################
######################################################################################
######################################################################################
# REST DOWN THERE IS JUST FOR PLOTTING
######################################################################################
######################################################################################

def from_id_width(id, width):
    return id % width, id // width


def draw_tile(graph, id, style, width):
    r = "."
    if ('number' in style
          and id in style['number']
          and style['number'][id] != float('inf')):
        r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = "\u2192"
        if x2 == x1 - 1: r = "\u2190"
        if y2 == y1 + 1: r = "\u2193"
        if y2 == y1 - 1: r = "\u2191"
    if 'start' in style and id == style['start']:
        r = "A"
    if 'goal' in style and id == style['goal']:
        r = "Z"
    if 'path' in style and id in style['path']:
        r = "@"
    if id in graph.walls: r = "#" * width
    return r


def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width),
                  end="")
        print()


def grid_from_string(string):#This function translates the input into a graph
    """
    Construct a SquareGrid from a string representation

    Representation:
    . - a passable square
    A - the start position
    Z - the goal position
    # - an unpassable square (a wall)

    Args:
        :type string: str

    Returns a 3-tuple: (g: SquareGrid, start: Tuple, end: Tuple)

    """
    assert string.count('A') == 1, "Cant have more than 1 start position!"
    assert string.count('Z') == 1, "Cant have more than 1 end position!"
    lines = [l.strip() for l in string.split('\n') if l.strip()]
    g = grid.SquareGrid(len(lines[0]), len(lines))
    start, end = None, None
    for row, line in enumerate(lines):
        for col, char in enumerate(line):
            if char == grid.WALL:
                g.walls.add((col, row))
            if char == 'A':
                start = (col, row)
            if char == 'Z':
                end = (col, row)
    assert start is not None
    assert end is not None
    return g, start, end



