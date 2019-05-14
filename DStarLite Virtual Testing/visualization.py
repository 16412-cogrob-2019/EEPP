import matplotlib.pyplot as plt
from numpy import dtype
import numpy as np
from numpy import Inf
from Astar import astar
from psetSolutions import heuristicwithcurrents, costwithcurrents, blurRiskField, costwithrisk, computeunitvector, cost_no_current_no_risk, heuristic_no_current_no_risk, heuristicwithrisk
from matplotlib.pyplot import pause

# class for visualizing the environment, mission, path, the problem, etc.
class VISUALIZATION:

    def __init__(self,auv,env):
        self.fig, self.ax = plt.subplots()
        self.AUV = auv
        self.Environment = env
        self.ax.set_aspect('equal', 'box')
        self.AlgorithmModel = {'runs': 0, 'graph': None, 'DStarLite':{}}

    # function for displaying the reef
    def ShowReef(self):
        if self.Environment.showcontours == True:
            CS = plt.contourf(self.Environment.X, self.Environment.Y, self.Environment.Reef(self.Environment.X,self.Environment.Y), 8, alpha=.75, cmap='gray')
            plt.colorbar(CS)
        else:
            self.ax.set_xlim(left=0, right=100)
            self.ax.set_ylim(bottom=0, top=100)
            self.ax.plot(self.AUV.origin[0],self.AUV.origin[1],'ko')
            self.ax.plot(self.AUV.goal[0],self.AUV.goal[1],'g*')

    # function for showing the obstacles
    def ShowObstacles(self,depth):
        r = np.vectorize(self.Environment.Reef)               
        self.ax.contourf(self.Environment.X, self.Environment.Y, r(self.Environment.X, self.Environment.Y),levels = [depth,Inf], colors = 'red')
    
    # function for showing the unknown regions
    def ShowUnknownRegions(self):
        from matplotlib import path as mpath
        import matplotlib.patches as patches

        for risk_key in self.Environment.UnknownRegions:
            unknown_region = patches.PathPatch(mpath.Path(self.Environment.UnknownRegions[risk_key]), facecolor='red', lw=0.0001, alpha = risk_key)
            self.ax.add_patch(unknown_region)
    
    # function for showing the risk
    def ShowRisk(self):
        rgba_colors = np.zeros((len(self.Environment.X.flatten()),4))
        # for red the first column needs to be one
        rgba_colors[:,0] = 1.0
        # the fourth column needs to be your alphas
        rgba_colors[:, 3] = np.transpose(self.Environment.RiskField).flatten()
        self.ax.scatter(self.Environment.X.flatten(), self.Environment.Y.flatten(), 0.1, color=rgba_colors)

    # function for showing the current
    def ShowCurrent(self):
        strengths = np.power(np.power(self.Environment.CurrentField_x,2)+np.power(self.Environment.CurrentField_y,2),0.5)
        strengths = strengths/np.max(strengths)
        self.ax.streamplot(self.Environment.X, self.Environment.Y, self.Environment.CurrentField_x, self.Environment.CurrentField_y,color='blue', linewidth=0.5*strengths)

    def Swim(self,plt,algo):
        from _protoTkinter_ import fctCAST
        fctCAST(self,plt,algo)
    
    # function for sending the AUV on its exploration mission
    def Explore(self):
        import numpy as np
        import matplotlib.patches as patches
        import matplotlib.path as mpath
        import matplotlib.animation as animation
        from matplotlib.transforms import Affine2D
        import copy

        # update coordinates of vehicle for animation
        verts = copy.deepcopy(self.AUV.verts0)
        codes = copy.deepcopy(self.AUV.codes0)
        barpath = mpath.Path(verts, codes)
        agent = patches.PathPatch(barpath, facecolor='yellow', alpha=1, linewidth=0.5)

        # update coordinates of obstacle contour at vehicle depth
        self.ShowObstacles(-9.5)
        pathline, = self.ax.plot(self.AUV.origin[0], self.AUV.origin[1], color='k', linewidth=2)
        def init():
            self.ax.add_patch(agent)

            return []

        def animationManage(z):
            animateAUV(z)
            AnimateObstacleContour(z)
            return []
    
        # function for animating the AUV
        def animateAUV(z):
            
            ## 
            sensor_range = 5
            obstacle_detected = self.Environment.collision_checker(self.AUV.path[z].position[0],self.AUV.path[z].position[1],self.AUV.path[z].position[2],sensor_range) 
            if  obstacle_detected == True:

                # stop the animation
                onClick()
                       
                # update RiskField by setting it to 1 at that point (self.AUV.path[z][0],self.AUV.path[z][1])
                self.Environment.RiskField[self.AUV.path[z].position[0],self.AUV.path[z].position[1]]
                    
                # call PlanPath again with new start point
                global pause
                pause = False
                self.AUV.PlanPath(alg = "A*", cost = costwithcurrents, heuristic = heuristicwithcurrents, env = self.Environment, vis=None,  alpha=1) # TODO: needs to be rearchitected
                
                # call Animation again with new path
                self.Explore() 
                    
            else:
            
                # turn the vehicle so it points tangential to its path between the previous and the next point
                orientation = np.rad2deg(np.arctan2(self.AUV.path[z].V_AUV[1],self.AUV.path[z].V_AUV[0]))
                rot_coordinates = Affine2D().rotate_deg(orientation).transform(self.AUV.verts0)
    
                # coordinate transformation: broadcast coordinates to all vehicle vertices
                for i, vert0 in enumerate(self.AUV.verts0):
                    for j, coordinate in enumerate(vert0):
                        verts[i][j] = rot_coordinates[i][j] + self.AUV.path[z].position[j]
    
                barpath = mpath.Path(verts, codes)
                pat = patches.PathPatch(barpath, facecolor='yellow', alpha=1, linewidth=0.5)
                pathline.set_data([node.position[0] for node in self.AUV.path[0:z]], [node.position[1] for node in self.AUV.path[0:z]])
                return pat,pathline

        # function for displaying the vehicle's changes in orientation
        def AnimateObstacleContour(z):
            del self.ax.collections[-1]
            self.ShowObstacles(self.AUV.path[z].position[2])
            return

        def onClick():
            global pause
            pause = True
                    
        self.fig.canvas.mpl_connect('button_press_event', onClick)
        return animation.FuncAnimation(self.fig, animationManage, init_func = init, frames=len(self.AUV.path), repeat=True, blit=True) # interval=1

    def show(self):
        self.ax.set_aspect('equal', 'box')
        plt.show()
