import numpy as np
from utils import dist
from scipy import ndimage

def heuristicwithcurrents(node1, node2, AUVspeed, max_strength=1):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/(AUVspeed + max_strength)
    return timetodestination 
    ### END SOLUTION

def costwithcurrents(node1, node2, AUVspeed, alpha):
    ### BEGIN SOLUTION
    S = AUVspeed
    #Use the current at the first node (could also use the average between node1 and node2)"
    V_current = node1.current

    #Unit vector from node1 to node2: use only x,y components
    s = np.array(node2.position)-np.array(node1.position)
    s = s[0:2].flatten()
    ns = np.linalg.norm(s)
    if ns < 1e-10:
        return (0.0, np.array((0.0,0.0)))
    s = s/ns


    c = (V_current[0]*s[1]-V_current[1]*s[0])

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
    
    return float(riskfactor*timeToDestination), V_AUV
    ### END SOLUTION

def blurRiskField(RiskField, sigma):
    """
    > RiskField is a 2D array of values r_ij = r(x_j, y_i)
    > sigma is the Gaussian standard deviation in units of Dgrid spacing
    """
    ### BEGIN SOLUTION
    G = ndimage.gaussian_filter(RiskField, sigma, mode='reflect', cval=0.0, truncate=4.0)
    return G
    ### END SOLUTION

def costwithrisk(node1, node2, AUVspeed, alpha):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUVspeed
    riskfactor = (1/(1-alpha*node2.risk + 1e-10))
    
    s = computeunitvector(node1,node2)
    V_AUV = s*AUVspeed
    
    return riskfactor*timetodestination, V_AUV
    ### END SOLUTION

# function that returns the unit vector pointing from node1 to node2
def computeunitvector(node1, node2): 
    s = np.array(node2.position)-np.array(node1.position)
    s = s[0:2].flatten()
    ns = np.linalg.norm(s)
    if ns < 1e-10:
        return np.array((0,0))
    s = s/ns
    return s

def cost_no_current_no_risk(node1, node2, AUV_speed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUV_speed
    s = computeunitvector(node1,node2)
    V_AUV = s*AUV_speed
    return timetodestination, V_AUV
    ### END SOLUTION

def heuristic_no_current_no_risk(node1, goal_node, AUV_speed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,goal_node)/AUV_speed
    return timetodestination
    ### END SOLUTION
    
def heuristicwithrisk(node1, node2, AUVspeed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUVspeed
    return timetodestination 
    ### END SOLUTION