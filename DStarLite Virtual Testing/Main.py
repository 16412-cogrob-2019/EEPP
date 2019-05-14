# DEPENDENCIES
from environment import ENVIRONMENT
from auv import AUV
from visualization import VISUALIZATION
from mission import MISSION
import matplotlib.pyplot as plt
from psetSolutions import blurRiskField

import time
start = time.time()

# define underwater exploration mission
UEXP = MISSION(discretization_distance=1, worldsize_x=100, worldsize_y=100)

# define autonomous underwater vehicle & assign vehicle parameters
AUV1 = AUV()

# allocate vehicle to a mission
setattr(AUV1, 'mission', UEXP)

# set vehicle parameters
AUV1.origin = (16,24,-9.5) # simple
AUV1.goal =  (3,30,-9.5) # simple
#AUV1.origin = (3,17,-9.5) # simple
#AUV1.goal =  (10,40,-9.5) # simple
#AUV1.origin = (70,100,-9.5) # advanced
#AUV1.goal =  (60,40,-9.5) # advanced
AUV1.speed = 2.0

# define our visualization output & create it
ENV1 = ENVIRONMENT(UEXP, ReefFunction='reef')
ENV1.UnknownRegions = { \
                       0.8: [(50, 15), (43, 25), (80, 25), (88, 19), (90,18)], \
                       0.4: [(80, 84), (95, 80), (95, 92), (76, 95)], \
                       0.1: [(11, 8), (40, 0), (40, 17), (11, 11)] \
                       }
ENV1.RiskField = ENV1.GenerateRiskField()
sigma = 1.5
ENV1.RiskField = blurRiskField(ENV1.RiskField, sigma)
ENV1.CurrentField_x, ENV1.CurrentField_y =  ENV1.GenerateCurrentField(type="whirlpool", max_strength=1)

VIS1 = VISUALIZATION(AUV1,ENV1)
VIS1.ShowReef()
# #VIS1.ShowCurrent()
VIS1.ShowRisk()
plt.savefig('foo.png')
plt.show(block=False)
plt.close()

AUV1.ReleaseAndExplore(VIS1, plt)

plt.show()