import matplotlib.pyplot as plt
from numpy import dtype
from Astar import astar

class ENVIRONMENT:

    def __init__(self,Mission = None, ReefFunction=None, UnknownRegions={}, discretization_x=101, discretization_y=101, blur = False):
        import numpy as np
        # set default values
        self.showcontours = None
        self.mission = Mission
        self.Dimension_x = self.mission.worldsize_x
        self.Dimension_y = self.mission.worldsize_y
        self.Reef = self.SetReef(ReefFunction)
        self.Current = [0.3,1.1,10]
        self.discretization_x = self.mission.discretization
        self.discretization_y = self.mission.discretization
        self.UnknownRegions = UnknownRegions
#         self.UnknownElements = [(55,50),(55,51),(55,52),(55,53),(55,54),(55,55),\
#                                 (56,50),(56,51),(56,52),(56,53),(56,54),(56,55),\
#                                 (57,50),(57,51),(57,52),(57,53),(57,54),(57,55),\
#                                 (58,50),(58,51),(58,52),(58,53),(58,54),(58,55),\
#                                 (59,50),(59,51),(59,52),(59,53),(59,54),(59,55),\
#                                 (60,50),(60,51),(60,52),(60,53),(61,54),(62,55),\
#                                 (61,50),(61,51),(61,52),(61,53),(61,54),(61,55),\
#                                 (62,50),(62,51),(62,52),(62,53),(62,54),(62,55),\
#                                 (63,50),(63,51),(63,52),(63,53),(63,54),(63,55),\
#                                 (64,50),(64,51),(64,52),(64,53),(64,54),(64,55),\
#                                 (65,50),(65,51),(65,52),(65,53),(65,54),(65,55),\
#                                 (66,50),(66,51),(66,52),(66,53),(66,54),(66,55),\
#                                 (67,50),(67,51),(67,52),(67,53),(67,54),(67,55),\
#                                 (68,50),(68,51),(68,52),(68,53),(68,54),(68,55)] 
        
        # simple
        self.UnknownElements = [(0,20),(0,21),(0,22),(0,23),(0,24),(0,25),\
                                (1,20),(1,21),(1,22),(1,23),(1,24),(1,25),\
                                (2,20),(2,21),(2,22),(2,23),(2,24),(2,25),\
                                (3,20),(3,21),(3,22),(3,23),(3,24),(3,25),\
                                (4,20),(4,21),(4,22),(4,23),(4,24),(4,25),\
                                (5,20),(5,21),(5,22),(5,23),(5,24),(5,25),\
                                (6,20),(6,21),(6,22),(6,23),(6,24),(6,25),\
                                (7,20),(7,21),(7,22),(7,23),(7,24),(7,25),\
                                (8,20),(8,21),(8,22),(8,23),(8,24),(8,25),\
                                (9,20),(9,21),(9,22),(9,23),(9,24),(9,25),\
                                (10,20),(10,21),(10,22),(10,23),(10,24),(10,25),\
                                (11,20),(11,21),(11,22),(11,23),(11,24),(11,25),\
                                (12,20),(12,21),(12,22),(12,23),(12,24),(12,25),\
                                (13,20),(13,21),(13,22),(13,23),(13,24),(13,25)] 
        
        # generate Dgrid
        self.x = np.linspace(0, self.Dimension_x, self.discretization_x)
        self.y = np.linspace(0, self.Dimension_y, self.discretization_y)
        self.X, self.Y = np.meshgrid(self.x, self.y)


        # generate risk and current fields
        self.RiskField = self.GenerateRiskField()
        self.CurrentField_x, self.CurrentField_y = self.GenerateCurrentField(type="none")

        # blur if requested
        if blur:
            self.Blur()

    # function for blurring the risk field
    def Blur(self):
        self.blur=True
        self.std_dev = 0.0 # up to 1.5 possible
        # blur risk and current fields
        self.RiskField = self.BlurField(self.RiskField, sigma = self.std_dev, dx = self.x[1] - self.x[0], dy = self.y[1] - self.y[0])
        self.CurrentField_x = self.BlurField(self.CurrentField_x, sigma = self.std_dev, dx = self.x[1] - self.x[0], dy = self.y[1] - self.y[0])
        self.CurrentField_y = self.BlurField(self.CurrentField_y, sigma = self.std_dev, dx = self.x[1] - self.x[0], dy = self.y[1] - self.y[0])
    
    def actualRisk(self,x,y,z):
        return self.RiskField[x,y]

    def ReturnRisk(self,x,y,z):
        x = float(x)
        y = float(y)
        z = float(z)
        from matplotlib import path as mpath
        risk = 0
        ctr = 0
        # step: loop over all unknown regions
        for risk_key in self.UnknownRegions:
            unknown_region = self.UnknownRegions[risk_key]
            region = mpath.Path(unknown_region)
            if region.contains_points([(x, y)]) == True:
                risk += risk_key
                ctr +=1

        # subtract independence P(A and B) = P(A) + P(B) - P(A U B)
        if ctr > 1:
            risk = risk * 0.9**ctr
        # step: check hard collisions with reef (tolerance could be set here, e.g. tol = 0.2)
        if self.collision_checker(x, y, z, tol = 0.0) == True:
            risk += 1
        else: pass

        # step: restrict risk to a maximum of 1
        if risk > 1:
            risk = 1
        else: pass

        return risk

    # function for generating a 2-d vector field for currents
    def GenerateCurrentField(self, type = "whirlpool", max_strength=1):
        import numpy as np
        
        # distinguishing current types
        if type == "none":
            CurrentField_x = np.zeros((len(self.x),len(self.y)))
            CurrentField_y = np.zeros((len(self.x),len(self.y)))
        elif type == "whirlpool":
            X,Y = self.X, self.Y
            lenx = len(X)
            leny = len(Y)
            ind = np.meshgrid(np.arange(-lenx/2,lenx/2),np.arange(-leny/2,leny/2))
            direction = -np.ones(lenx*leny).reshape(lenx, leny)*np.arctan2(ind[0],ind[1])
            CurrentField_x, CurrentField_y = max_strength * np.cos(direction), max_strength * np.sin(direction)
        elif type == "uniformX":
            CurrentField_x = max_strength*np.ones_like(self.X)
            CurrentField_y = 0*np.ones_like(self.X)
        elif type == "uniformY":
            CurrentField_x = 0*np.ones_like(self.X)
            CurrentField_y = max_strength*np.ones_like(self.X)
        else:
            CurrentField_x = np.zeros((len(self.x),len(self.y)))
            CurrentField_y = np.zeros((len(self.x),len(self.y)))
            for i,x in enumerate(self.x):
                for j,y in enumerate(self.y):
                    CurrentField_x[i][j] = x
                    CurrentField_y[i][j] = y
        return CurrentField_x, CurrentField_y

    # function for generating a 2-d array for risk
    def GenerateRiskField(self):
        import numpy as np
        RiskField = np.zeros((len(self.x),len(self.y)))
        for i,x in enumerate(self.x):
            for j,y in enumerate(self.y):
                RiskField[i][j] = self.ReturnRisk(x=x, y=y, z=-9.5)
        return RiskField

    def BlurField(self,r,sigma,dx,dy):
        import scipy.ndimage
        """
        > r is a 2D array of values r_ij = r(x_j, y_i) (could be risk, current, etc.)
        > sigma is the Gaussian std.dev. in meters (in the field)
        > dx, dy are Dgrid spacing in meters (used to scale sigma)
        """
        sigma = (sigma/dx, sigma/dy)
        G = scipy.ndimage.gaussian_filter(r, sigma, mode='reflect', cval=0.0, truncate=4.0)
        return G

    # function for setting the reef for the environment
    def SetReef(self,ReefFunction):
        import numpy as np
        def StandardReefFunction(x, y):
                x = (x-0.5 *self.Dimension_x)/15
                y = (y-0.5 *self.Dimension_y)/15
                return (1.0 - x / 2.0 + x ** 5.0 + y ** 3.0) * np.exp(-x ** 2.0 - y ** 2.0) - 10.0
        
        def DeepPoolFunction(x, y):
            return -10
        
        # in case no reef function is specified take a standard reef function
        if ReefFunction == None or ReefFunction == 'reef':
            ReefFunction = StandardReefFunction
            self.showcontours = True
        elif ReefFunction == 'pool':
            ReefFunction = DeepPoolFunction
            self.showcontours = False
        else: pass

        return ReefFunction

    # function that checks for collisions
    def collision_checker(self,x,y,z,tol):
        """
        Checks collisions with the reef
        Assumptions: The reef surface is the actual ground and not a layer. Hence, so far we are not including exploring caves (but maybe later we can).
        """
        from numpy import inf
        import bisect
        if z <= (self.Reef(float(x),float(y))+tol):
        # if (bisect.bisect_left([-inf,max(self.Reef(x,y) - tol, self.Reef(x,y) + tol)], z)) == 1:
            return True
        else: return False
