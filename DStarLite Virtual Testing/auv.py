from numpy import dtype
from Astar import astar
from dijkstra import global_planner
import numpy as np
import sys
import math as m
from GraphUtils import GetNode
from PriorityQueue import PriorityQueue
import networkx as nx

def vCalc(G,source,target):
        S = G.graph['AUVmaxspeed']
        #Use the current at the first node (could also use the average between node1 and node2)"
        V_current = source.current
    
        #Unit vector from node1 to node2: use only x,y components
        s = np.array(target.position)-np.array(source.position)
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

def h(G,source,target):
    """
    Gives estimates of travel cost between two nodes
    """  
    if source != target:
        # calculate distance
        dist = m.hypot(target.position[0] - source.position[0], target.position[1] - source.position[1])
        maxcurrent = G.graph['maxcurrent']
        speed_exaggerated = G.graph['AUVmaxspeed'] + np.linalg.norm(maxcurrent)
        costestimate = dist/speed_exaggerated
        source.h = {target: costestimate}
    else: 
        source.h = {target: 0.}
    return source.h[target]

def c(G,source,target):
    """
    Gives edge cost
    """
    if source != target: 
        #Unit vector from node1 to node2: use only x,y components
        s = np.array(target.position)-np.array(source.position)
        s = s[0:2].flatten()
        ns = np.linalg.norm(s)
        if ns < 1e-10:
            return (0.0, np.array((0.0,0.0)))
        s = s/ns
        alpha = G.graph['alpha']
        V_AUV = vCalc(G,source, target)
        V_current = source.current
        source.V_AUV = V_AUV
     
        # compute the speed we are traveling in the target direction, i.e. from node1 to node2
        SpeedInTargetDirection = np.dot(V_AUV,s) + np.dot(V_current,s)
     
        # compute the time it will take to travel from node1 to node2 - handle edge case that 0 speed=inf time
        dist = m.hypot(target.position[0] - source.position[0], target.position[1] - source.position[1])
        time = dist/SpeedInTargetDirection if SpeedInTargetDirection != 0 else 0
        timeToDestination = (time if time > 0  else np.inf)
        riskfactor = 1/(1-alpha*target.risk + 1e-10)
         
        # calculate cost-to-go
        edgecost = float(riskfactor*timeToDestination)
        source.c = {target: edgecost}
    else: 
        source.c = {target: 0.}
    return source.c[target]

class AUV:
    def __init__(self):
        import matplotlib.path as mpath
        from matplotlib.transforms import Affine2D
        from copy import deepcopy

        self.origin = (None,None,None)
        self.goal = (None,None,None)
        self.path = None
        self.mission = None
        self.speed = 10

        # define AUV body [m] (those coordinates are centered on (0,0))
        nose    = [0.    , +2.0]
        noseL   = [-0.25 , +1.5]
        noseR   = [+0.25 , +1.5]
        tailL   = [-0.25 , -1.0]
        tailR   = [+0.25 , -1.0]
        path_data = [
            (mpath.Path.MOVETO, nose),
            (mpath.Path.LINETO, noseR),
            (mpath.Path.LINETO, tailR),
            (mpath.Path.LINETO, tailL),
            (mpath.Path.LINETO, noseL),
            (mpath.Path.CLOSEPOLY, noseL)
            ]

        # store vehicle body data
        self.verts0 = Affine2D().rotate_deg(-90).transform( np.array(list(zip(*path_data))[1], float) )
        self.codes0 = list(zip(*path_data))[0]
        self.height = 0.30 # vehicle diameter (cylindrical body)
    
    # function for returning vehicle state
    def StateInfo(self):
        print('origin: ',self.origin)
        print('goal: ', self.goal)

    def ReleaseAndExplore(self,VIS,plt):
        from GraphUtils import AbstractToGraph
        from _BringToLife_ import BringToLife # TODO: make this a method of the environment class (only for debugging excluded)
        
        # initialize meta parameters for algorithms
        VIS.AlgorithmModel['runs'] = 0 
        #algo= 'DstarLite'
        algo= 'A*'
        BringToLife(VIS,plt,algo,vCalc,c,h)
    
    def PlanPath(self, env, alg, cost, heuristic, vis, alpha):
        '''
        This is our intelligent function that comes up with a path!
        '''

        collision_tolerance = 0.2

        if self.origin == None or self.goal == None:
            raise Exception("Path planner did not plan any path. Please specify the AUV's origin and goal first!")

        # select path planning algorithm/method
        if alg == "A*":
            self.path = astar(auv = self, env=env, cost=cost, heuristic=heuristic, vis=vis, alpha = alpha)
        elif alg == "Dijkstra":
            self.path = global_planner(auv=self, env=env, cost=cost, vis=vis, alpha=alpha)
        else:
            speed_x = 0.3
            speed_y = 0.20
            speed_z = 0.00
            vehicle_path = np.zeros((self.mission.discretization, 3))
            for timepoint in range(self.mission.discretization):
                x = self.origin[0] + speed_x * timepoint
                y = self.origin[1] + speed_y * timepoint
                z = self.origin[2] + speed_z * timepoint

                # check whether coordinates are valid or whether they cause a crash
                if env.collision_checker(x,y,z,collision_tolerance) == True:
                    sys.exit("Path planner failed. Your vehicle would have collided at X = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z))

                vehicle_path[timepoint] = np.array([x,y,z])
            setattr(self,'path',vehicle_path)
            print(vehicle_path)
