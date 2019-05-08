import numpy as np
import heapq
import random

"""
Return the distance from node1 to node2
"""
def dist(node1, node2):
    return (((node1.position[0] - node2.position[0]) ** 2) + ((node1.position[1] - node2.position[1]) ** 2)) **0.5

def compute_unit_vector(node1, node2):
    # Unit vector pointing from node1 to node2
    s = np.array(node2.position)-np.array(node1.position)
    s = s[0:2].flatten()
    ns = np.linalg.norm(s)
    if ns < 1e-10:
        return np.array((0,0))
    s = s/ns
    return s

def heuristic(node1, node2, AUV_speed):
    # TODO: make max_current the actual max current
    max_current = 2.0
    speed_exaggerated = AUV_speed + max_current
    time_to_destination = dist(node1, node2)/speed_exaggerated
    return time_to_destination

def cost_function(node1, node2, AUV_speed, alpha):
    S = AUV_speed
    #Use the current at the first node (could also use the average between node1 and node2)"
    V_current = node1.current

    #Unit vector from node1 to node2: use only x,y components
    s = np.array(node2.position) - np.array(node1.position)
    s = s[0:2].flatten()
    ns = np.linalg.norm(s)
    if ns < 1e-10:
        return (0, np.array((0,0)))
    s = s/ns

    c = -(V_current[0]*s[1]-V_current[1]*s[0])

    # Compute V_AUV - the required AUV velocity direction to get us from node1 to node2, accounting for the current
    # uses quadratic formula, so have to check if we want the negative or positive solution
    V_AUVy = (np.sqrt((s[0]**2)*(s[1]**2)*(S**2)+ (s[1]**4)*(S**2) -(s[1]**2)*c**2) -s[0]*c)

    V_AUVpp = np.array((np.sqrt(S**2 - V_AUVy**2),V_AUVy))
    V_AUVpn = np.array((np.sqrt(S**2 - V_AUVy**2),-V_AUVy))
    V_AUVnp = np.array((-np.sqrt(S**2 - V_AUVy**2),V_AUVy))
    V_AUVnn = np.array((-np.sqrt(S**2 - V_AUVy**2),-V_AUVy))

    list = [V_AUVpp,V_AUVpn, V_AUVnp,V_AUVnn]

    V_AUV = list[np.argmax([np.dot(V+V_current,s) for V in list])]

    # compute the speed we are travelling in the target direction, i.e. from node1 to node2
    SpeedInTargetDirection = np.dot(V_AUV,s) + np.dot(V_current,s)

    # compute the time it will take to travel from node1 to node2 - handle edge case that 0 speed=inf time
    time = dist(node1,node2)/SpeedInTargetDirection if SpeedInTargetDirection != 0 else 0
    timeToDestination = (time if time > 0  else np.inf)

    riskfactor = (1/(1-alpha*node2.risk + 1e-10))

    return riskfactor*timeToDestination, SpeedInTargetDirection

"""
Priority Queue for use in A*
"""
class NodePriorityQueue:
    def __init__(self):
        self.elements = []
        self.maxlength = 0
        self.element_pos = set()

    def empty(self):
        return len(self.elements) == 0

    def put(self, node, priority):
        heapq.heappush(self.elements, ((priority, random.random()), node))
        self.element_pos.add(node.position)
        self.maxlength = max(self.maxlength,len(self.elements))

    def pop(self):
        item = heapq.heappop(self.elements)
        self.element_pos.remove(item[1].position)
        return item[1]

    def delete(self, pos):
        self.elements = [e for e in self.elements if e[1].position != pos]
        self.element_pos.remove(pos)
        heapq.heapify(self.elements)

    def get_node(self, pos):
        for e in self.elements:
            if e[1].position == pos:
                return e[1]
        return None

    def in_queue(self, pos):
        return pos in self.element_pos

    def __iter__(self):
        for key, node in self.elements:
            yield node
