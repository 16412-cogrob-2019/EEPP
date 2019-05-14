import networkx as nx
import intervals as I
import math as m
import numpy as np
from PriorityQueue import PriorityQueue
from utils import dist

def AbstractToGraph(obj):
    """Abstract a NetworkX graph for Path Planning from our Application"""
    # interfacing
    xmax = obj.Environment.Dimension_x 
    ymax = obj.Environment.Dimension_y 
    
    # Create nodes and edges
    edges = [ ( (x,y),(x+dx, y+dy) ) for dx in [-1,0,+1] for dy in [-1,0,+1] for x in range(xmax+1) for y in range(ymax+1) if x+dx in I.closed(0, xmax) and y+dy in I.closed(0, ymax) and (dx!=0 or dy!=0)]
    G = nx.DiGraph(alpha=0.5,AUVmaxspeed=2.0)
    G.add_edges_from(edges)
    
    # Assign node data (current and risk)
    attrs = {n: {'current':  (obj.Environment.CurrentField_x[ n[0] ][ n[1] ], obj.Environment.CurrentField_y[ n[0] ][ n[1] ]), 'risk': obj.Environment.RiskField[ n[0] ][ n[1] ], 'V_AUV': None, 'h': {}, 'c': {}} for n in G.nodes }
    nx.set_node_attributes(G, attrs)
    return G

def IDstarLite(G,start, goal, obj):
    """DstarLite algorithm implementation """
    """
    u: vertices
    s: nodes
    """
    def vCalc(source,target):
        S = G.graph['AUVmaxspeed']
        #Use the current at the first node (could also use the average between node1 and node2)"
        V_current = G.nodes[source]['current']
    
        #Unit vector from node1 to node2: use only x,y components
        s = np.array(target)-np.array(source)
        s = s[0:2].flatten()
        ns = np.linalg.norm(s)
        if ns < 1e-10:
            return (0.0, np.array((0.0,0.0)))
        s = s/ns
        c = (V_current[0]*s[1]-V_current[1]*s[0])
    
        # Compute V_AUV - the required AUV velocity direction to get us from node1 to node2, accounting for the current
        V_AUVy = (np.sqrt((s[0]**2)*(s[1]**2)*(S**2)+ (s[1]**4)*(S**2) -(s[1]**2)*c**2) -s[0]*c)
        V_AUVpp = np.array((np.sqrt(S**2 - V_AUVy**2),V_AUVy))
        V_AUVpn = np.array((np.sqrt(S**2 - V_AUVy**2),-V_AUVy))
        V_AUVnp = np.array((-np.sqrt(S**2 - V_AUVy**2),V_AUVy))
        V_AUVnn = np.array((-np.sqrt(S**2 - V_AUVy**2),-V_AUVy))
        l = [V_AUVpp,V_AUVpn, V_AUVnp,V_AUVnn]
        V_AUV = l[np.argmax([np.dot(V+V_current,s) for V in l])]
        return V_AUV
    
    def IdentifyAffectedEdges(node):
        """Returns the edges effected by a node data change """
        return G.in_edges(node) #+ G.out_edges(node)

    def c(source,target):
        """
        Gives edge cost
        """
        if source != target: 
            #Unit vector from node1 to node2: use only x,y components
            s = np.array(target)-np.array(source)
            s = s[0:2].flatten()
            ns = np.linalg.norm(s)
            if ns < 1e-10:
                return (0.0, np.array((0.0,0.0)))
            s = s/ns
            alpha = G.graph['alpha']
            V_AUV = vCalc(source, target)
            V_current = G.nodes[source]['current']
            G.nodes[source]['V_AUV'] = V_AUV
         
            # compute the speed we are travelling in the target direction, i.e. from node1 to node2
            SpeedInTargetDirection = np.dot(V_AUV,s) + np.dot(V_current,s)
         
            # compute the time it will take to travel from node1 to node2 - handle edge case that 0 speed=inf time
            dist = m.hypot(target[0] - source[0], target[1] - source[1])
            time = dist/SpeedInTargetDirection if SpeedInTargetDirection != 0 else 0
            timeToDestination = (time if time > 0  else np.inf)
            riskfactor = 1/(1-alpha*G.nodes[target]['risk'] + 1e-10)
             
            # calculate cost-to-go
            edgecost = float(riskfactor*timeToDestination)
            G.nodes[source]['c'][target] = edgecost
        else: 
            G.nodes[source]['c'][target] = 0.
        return G.nodes[source]['c'][target]
        
    def h(source,target):
        """
        Gives estimates of travel cost between two nodes
        """  
        if source != target:
            # calculate distance
            dist = m.hypot(target[0] - source[0], target[1] - source[1])
            maxcurrent = G.graph['maxcurrent']
            speed_exaggerated = G.graph['AUVmaxspeed'] + np.linalg.norm(maxcurrent)
            costestimate = dist/speed_exaggerated
            G.nodes[source]['h'][target] = costestimate
        else: 
            G.nodes[source]['h'][target] = 0.
        return G.nodes[source]['h'][target]
         
    def CalculateKey(s):
        return [ min(G.nodes[s]['g'],G.nodes[s]['rhs']) + h(start,s) + G.graph['km'], min(G.nodes[s]['g'],G.nodes[s]['rhs']) ]
         
    def UpdateVertex(u):
        if u != goal:
            G.nodes[u]['rhs'] = min( [c(u,successor) + G.nodes[successor]['g'] for successor in G.successors(u)] )
        if u in G.graph['U']:
            G.graph['U'].delete(u)
        if G.nodes[u]['g'] != G.nodes[u]['rhs']:
            G.graph['U'].put(u,CalculateKey(u))
    
    def ComputeShortestPath():
        debug_ctr = 0
        while G.graph['U'].TopKey() < CalculateKey(start) or G.nodes[start]['rhs'] != G.nodes[start]['g']:
            debug_ctr +=1
            G.graph['kold'] = G.graph['U'].TopKey()
            u = G.graph['U'].pop()
            if G.graph['kold'] < CalculateKey(u):
                G.graph['U'].put(u,CalculateKey(u))
            elif G.nodes[u]['g'] > G.nodes[u]['rhs']: 
                G.nodes[u]['g'] = G.nodes[u]['rhs']
                for s in G.predecessors(u):
                    UpdateVertex(s)
            else: 
                G.nodes[u]['g'] = np.inf
                for s in list(G.predecessors(u)) + [u]:
                    UpdateVertex(s)
    
    # always initialize as we never take a step while the algorithm is still running
    if G.graph['runs'] == 0: # then initialize
        G.graph['maxcurrent'] = max( nx.get_node_attributes(G,'current').values(),key= np.linalg.norm )
        G.graph['slast'] = start
        G.graph['U'] = PriorityQueue()
        G.graph['km'] = 0
        # initialize nodes
        for s in G.nodes:
            G.nodes[s]['rhs'] = np.inf
            G.nodes[s]['g'] = np.inf
        # initialize goal rhs value
        G.nodes[goal]['rhs'] = 0
        G.graph['U'].put(goal,CalculateKey(goal))
    else: # load the most recent values from algorithm model
        data_change = G.graph['updates']
        
        # Update algorithm model
        G.graph['km'] = G.graph['km'] + h(G.graph['slast'],start)
        G.graph['slast'] = start
        
        # update network according to environment changes
        for _,changes in data_change.items():
            for u,_ in changes.items():
                G.node[u]['risk'] = 1.0 # TODO: make this change outside of the algorithm to the graph directly ...
                for edge in G.in_edges(u):
                    UpdateVertex(edge[0])

    ComputeShortestPath()
    
    print("MaxQueueLength: ",G.graph['U'].maxlength)
    
    # Store algorithm model information in AUV storage 
    G.graph['runs'] += 1
    obj.AlgorithmModel['DstarLite']['graph'] = G
    
    # Return the path
    path = []
    cost = 0
    v = start
    while v != goal:
        # add contribution to path cost
        d = {k: c(v,k)+G.node[k]['g'] for k in G.successors(v)}
        nxt = min(d, key=d.get)
        # put node in path
        cost += c(v,nxt)
        path.append({'position': v, 'V_AUV': vCalc(v, nxt)})
        v = nxt
    path.append({'position': goal, 'V_AUV': np.zeros(2)})
    return path,cost,G  
        
            