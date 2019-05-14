import networkx as nx
import math as m
import numpy as np
from PriorityQueue import PriorityQueue
from utils import dist
from GraphUtils import GetNode
import time

def DStarLite(G,start, goal, obj,vCalcfct,cfct,hfct):
    """DstarLite algorithm implementation """
    """
    u: vertices
    s: nodes
    """
    
    def IdentifyAffectedEdges(node):
        """Returns the edges effected by a node data change """
        return G.in_edges(node)
         
    def CalculateKey(s):
        return [ min(s.g, s.rhs) + hfct(G,G.graph['start'],s) + G.graph['km'], min(s.g, s.rhs) ]
         
    def UpdateVertex(u):
        if u != G.graph['goal']:
            u.rhs = min( [cfct(G,u,successor) + successor.g for successor in G.successors(u)] )
        if u in G.graph['U']:
            G.graph['U'].delete(u)
        if u.g != u.rhs:
            G.graph['U'].put(u,CalculateKey(u))
    
    def ComputeShortestPath():
        debug_ctr = 0
        while G.graph['U'].TopKey() < CalculateKey(G.graph['start']) or G.graph['start'].rhs != G.graph['start'].g:
            a = list( G.graph['U'])
            debug_ctr +=1
            G.graph['kold'] = G.graph['U'].TopKey()
            u = G.graph['U'].pop()
            if G.graph['kold'] < CalculateKey(u):
                G.graph['U'].put(u,CalculateKey(u))
            elif u.g > u.rhs: 
                u.g = u.rhs
                for s in G.predecessors(u):
                    UpdateVertex(s)
            else: 
                u.g = np.inf
                for s in list(G.predecessors(u)) + [u]:
                    UpdateVertex(s)
    
    # Interface part: #TODO: This should be made a meta-level function
    if obj.AlgorithmModel['runs'] == 0:
        # initialize goal node to rhs value of 0
        G.graph['U'] = PriorityQueue()
        G.graph['km'] = 0
        G.graph['start'] = GetNode(G, 'position', (start[0],start[1]))
        goal = GetNode(G, 'position', (goal[0],goal[1]))
        goal.rhs = 0
        G.graph['goal'] = goal
        G.graph['alpha'] = 1
        G.graph['slast'] = GetNode(G, 'position', (G.graph['start'].position[0],G.graph['start'].position[1]))
        G.graph['U'].put( G.graph['goal'],[ min(G.graph['goal'].g, G.graph['goal'].rhs) + hfct(G,G.graph['start'],G.graph['goal']) + G.graph['km'], \
                                           min(G.graph['goal'].g, G.graph['goal'].rhs) ] )
 
        print("################",list(G.graph['U']))
    else: # load the most recent values from algorithm model
        G.graph['start'] = GetNode(G, 'position', (start[0],start[1]))
        G.graph['goal'] = GetNode(G, 'position', (goal[0],goal[1]))
         
        data_change = G.graph['updates']
         
        # Update algorithm model
        G.graph['km'] = G.graph['km'] + hfct(G,G.graph['slast'],G.graph['start'])
        G.graph['slast'] = G.graph['start']
         
        # update network according to environment changes
        for _,changes in data_change.items():
            for t,_ in changes.items():
                u = GetNode(G, 'position', t)
                u.risk = 1.0
                for edge in G.in_edges(u):
                    UpdateVertex(edge[0])

    # performance measurements
    ComputeShortestPath()
    
    # Store algorithm model information in AUV storage 
    obj.AlgorithmModel['graph'] = G
    
    # Return the path
    path = []
    cost = 0
    v = G.graph['start']
    while v != G.graph['goal']:
        # add contribution to path cost
        d = {k: cfct(G,v,k) + k.g for k in G.successors(v)}
        nxt = min(d, key=d.get)
        # put node in path
        cost += cfct(G,v,nxt)
        path.append(v)
        v = nxt
    path.append(G.graph['goal']) # append with V_AUV = np.zeros(2)
    return path,cost,G 