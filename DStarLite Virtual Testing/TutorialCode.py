# DEPENDENCIES
import numpy as np
from environment import ENVIRONMENT
import scipy.ndimage
from auv import AUV
from Astar import Node
from visualization import VISUALIZATION
from mission import MISSION

from utils import InspectReefData2D, InspectReefData3D, InspectReefData2DComplete
import matplotlib.pyplot as plt

size = 9

# set size of the plot
plt.rcParams['figure.figsize'] = [size, size]

#####################################
#####################################
#####################################

# function that returns the distance between two nodes
from utils import dist 

# function that returns the unit vector pointing from node1 to node2
def computeunitvector(node1, node2): 
    s = np.array(node2.position)-np.array(node1.position)
    s = s[0:2].flatten()
    ns = np.linalg.norm(s)
    if ns < 1e-10:
        return np.array((0,0))
    s = s/ns
    return s

#####################################
#####################################
#####################################

def cost_no_current_no_risk(node1, node2, AUV_speed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUV_speed
    s = computeunitvector(node1,node2)
    V_AUV = s*AUV_speed
    return timetodestination, V_AUV
    ### END SOLUTION

#####################################
#####################################
#####################################

def heuristic_no_current_no_risk(node1, goal_node, AUV_speed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,goal_node)/AUV_speed
    return timetodestination
    ### END SOLUTION

#####################################
#####################################
#####################################

# define underwater exploration mission
UEXP = MISSION(discretization_distance=1, worldsize_x=100, worldsize_y=100)

#####################################
#####################################
#####################################

ENV1 = ENVIRONMENT(UEXP, ReefFunction= "pool")

#####################################
#####################################
#####################################

# define autonomous underwater vehicle & assign vehicle parameters
AUV1 = AUV()

# allocate vehicle to a mission
setattr(AUV1, 'mission', UEXP)

# set vehicle parameters
AUV1.origin = (50, 50, -9.5)
AUV1.goal = (80, 50,-9.5)
AUV1.speed = 2

#####################################
#####################################
#####################################

# define our visualization output & create it
VIS1 = VISUALIZATION(AUV1,ENV1)
VIS1.ShowReef()

#####################################
#####################################
#####################################

# Plan the path and animate the search process
anim = AUV1.PlanPath(alg = "Dijkstra", cost=cost_no_current_no_risk, heuristic=None, env = ENV1, vis=VIS1, alpha=0)

#####################################
#####################################
#####################################

# need to reinstantiate the visualization class to get animations to show in notebook ...
VIS1 = VISUALIZATION(AUV1,ENV1)
VIS1.ShowReef()

# plan path for the pool test environment
AUV1.PlanPath(alg = "A*", cost = cost_no_current_no_risk, heuristic = heuristic_no_current_no_risk, env = ENV1, vis=None, alpha=0)
VIS1.video = VIS1.Explore()

#####################################
#####################################
#####################################

plt.rcParams['figure.figsize'] = [size, size]

ENV2 = ENVIRONMENT(UEXP, ReefFunction='reef')
VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)

#####################################
#####################################
#####################################

plt.rcParams['figure.figsize'] = [size, size]

ENV2 = ENVIRONMENT(UEXP, ReefFunction=None)

ENV2.UnknownRegions = { \
                       0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                       0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                       0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                       }
ENV2.RiskField = ENV2.GenerateRiskField()

VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)
VIS2.ShowRisk()
plt.show()

#####################################
#####################################
#####################################

def costwithrisk(node1, node2, AUVspeed, alpha):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUVspeed
    riskfactor = (1/(1-alpha*node2.risk + 1e-10))
    
    s = computeunitvector(node1,node2)
    V_AUV = s*AUVspeed
    
    return riskfactor*timetodestination, V_AUV
    ### END SOLUTION

#####################################
#####################################
#####################################

def heuristicwithrisk(node1, node2, AUVspeed):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/AUVspeed
    return timetodestination 
    ### END SOLUTION

#####################################
#####################################
#####################################

# define our visualization output & create it
VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)
VIS2.ShowRisk()

AUV1.origin = (30, 50, -9.5)
AUV1.goal = (90, 50,-9.5)

AUV1.PlanPath(alg = "A*", cost = costwithrisk, heuristic = heuristicwithrisk, env = ENV2, vis=None,  alpha=1)
VIS2.video = VIS2.Explore()

#####################################
#####################################
#####################################

def blurRiskField(RiskField, sigma):
    """
    > RiskField is a 2D array of values r_ij = r(x_j, y_i)
    > sigma is the Gaussian standard deviation in units of Dgrid spacing
    """
    ### BEGIN SOLUTION
    G = scipy.ndimage.gaussian_filter(RiskField, sigma, mode='reflect', cval=0.0, truncate=4.0)
    return G
    ### END SOLUTION
	
#####################################
#####################################
#####################################

ENV2 = ENVIRONMENT(UEXP, ReefFunction=None)

ENV2.UnknownRegions = { \
                       0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                       0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                       0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                       }
ENV2.RiskField = ENV2.GenerateRiskField()

sigma = 1.5
ENV2.RiskField = blurRiskField(ENV2.RiskField, sigma)
# define our visualization output & create it
VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)
VIS2.ShowRisk()

AUV1.origin = (30, 50, -9.5)
AUV1.goal = (90, 50,-9.5)

AUV1.PlanPath(alg = "A*", cost = costwithrisk, heuristic = heuristicwithrisk, env = ENV2, vis=None,  alpha=1)
VIS2.video = VIS2.Explore()
HTML(VIS2.video.to_html5_video())

#####################################
#####################################
#####################################

ENV2 = ENVIRONMENT(UEXP, ReefFunction=None)

ENV2.UnknownRegions = { \
                       0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                       0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                       0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                       }
ENV2.RiskField = ENV2.GenerateRiskField()
ENV2.CurrentField_x, ENV2.CurrentField_y =  ENV2.GenerateCurrentField(type="whirlpool", max_strength=1)
VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)
VIS2.ShowRisk()
VIS2.ShowCurrent()

#####################################
#####################################
#####################################

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
    print(V_AUV)

    # compute the speed we are travelling in the target direction, i.e. from node1 to node2
    SpeedInTargetDirection = np.dot(V_AUV,s) + np.dot(V_current,s)

    # compute the time it will take to travel from node1 to node2 - handle edge case that 0 speed=inf time
    time = dist(node1,node2)/SpeedInTargetDirection if SpeedInTargetDirection != 0 else 0
    timeToDestination = (time if time > 0  else np.inf)
    
    riskfactor = (1/(1-alpha*node2.risk + 1e-10))
    
    return float(riskfactor*timeToDestination), V_AUV
    ### END SOLUTION
	
#####################################
#####################################
#####################################

def heuristicwithcurrents(node1, node2, AUVspeed, max_strength=1):
    ### BEGIN SOLUTION
    timetodestination = dist(node1,node2)/(AUVspeed + max_strength)
    return timetodestination 
    ### END SOLUTION
	
#####################################
#####################################
#####################################

ENV2 = ENVIRONMENT(UEXP, ReefFunction=None)

ENV2.UnknownRegions = { \
                       0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                       0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                       0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                       }
ENV2.RiskField = ENV2.GenerateRiskField()
ENV2.CurrentField_x, ENV2.CurrentField_y =  ENV2.GenerateCurrentField(type="whirlpool", max_strength=1)
VIS2 = VISUALIZATION(AUV1,ENV2)
VIS2.ShowReef()
VIS2.ShowObstacles(-9.5)
VIS2.ShowRisk()
VIS2.ShowCurrent()


AUV1.origin = (30, 70, -9.5)
AUV1.goal = (90, 50,-9.5)

AUV1.PlanPath(alg = "A*", cost = costwithcurrents, heuristic = heuristicwithcurrents, env = ENV2, vis=None,  alpha=1)
VIS2.video = VIS2.Explore()
HTML(VIS2.video.to_html5_video())