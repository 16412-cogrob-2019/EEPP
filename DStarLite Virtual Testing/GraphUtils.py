import networkx as nx
import intervals as I
import numpy as np
from scipy.special._precompute.gammainc_asy import eta

class SearchNode:
    """A generic node class for graph search algorithms"""
    def __init__(self,position,risk,current,h=None,c=None,rhs=None,g=None):
        self.position = position
        self.risk = risk
        self.current = current
        self.h = h # TODO: Hook up with functions
        self.c = c # TODO: Hook up with functions
        self.rhs = rhs
        self.g = g
        self.V_AUV = None

def GetNode(G,SearchAttribute,val):
    node = [n for n in G.nodes if getattr(n, SearchAttribute) == val][0]
    return node

def ReturnCurrent(obj,position):
    return (obj.Environment.CurrentField_x[ position[0] ][ position[1] ],obj.Environment.CurrentField_y[ position[0] ][ position[1] ])

def ReturnRisk(obj,position):
    return obj.Environment.RiskField[ position[0] ][ position[1] ]

def AbstractToGraph(obj):
    """Abstracts the environment to a graph so that path can be planned"""
    # read environmental limits
    xmax = obj.Environment.Dimension_x 
    ymax = obj.Environment.Dimension_y
    
    # create graph
    G = nx.DiGraph(alpha=1,AUVmaxspeed=2.0) # TODO: hand over the parameters
    
    #create raw edges between coordinates
    raw_edges = [ ((x,y),(x+dx,y+dy)) for dx in [-1,0,+1] for dy in [-1,0,+1] for x in range(xmax+1) for y in range(ymax+1) if x+dx in I.closed(0, xmax) and y+dy in I.closed(0, ymax) and (dx!=0 or dy!=0)]

    # create nodes
    nodes = {(x,y): SearchNode(position=(x,y),risk=ReturnRisk(obj,(x,y)),current=ReturnCurrent(obj,(x,y)),g=np.inf,rhs=np.inf) for x in range(xmax+1) for y in range(ymax+1)}
    
    # make graph edges from raw edges and nodes
    edges = [(nodes[e[0]],nodes[e[1]]) for e in raw_edges] 
    G.add_edges_from(edges)
    
    # identify the maxcurrent in the model
    G.graph['maxcurrent'] = max( [n.current for n in G.nodes],key=np.linalg.norm )
    
    return G

        
class Node: #TODO: Make that class more generic!
    """A node class for A*"""
    def __init__(self, parent=None, position=None, U= None, V=None, current_cost=np.inf):
        self.parent = parent
        self.position = position
        self.g = None
        self.h = None
        self.f = None
        self.current = currentAt(self, U, V)
        self.cost = current_cost
        self.transition_cost = None
        self.V_AUV = None
        self.V_AUVs = None
        self.risk = None
    def __str__(self):
        return str(self.position)
    def __repr__(self):
        return str(self.position)
    def __eq__(self, other):
        return self.position == other.position
    def update_transitions(self, edges):
        self.transition_cost = edges
    def update_current_cost(self, new_cost):
        self.cost = new_cost
    def update_vauvs(self, vauvs):
        self.V_AUVs = vauvs

def currentAt(node, U, V):
    """
    Compute current at the given node, using matrices U and V of X,Y velocity components respectively
    """
    if U is None or V is None:
        return None
    return np.array((U[node.position[1], node.position[0]], V[node.position[1], node.position[0]]))