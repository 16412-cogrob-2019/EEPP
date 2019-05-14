from tkinter import *
from visualization import VISUALIZATION
from math import atan,degrees,radians
import cmath
import numpy as np
from PIL import Image
from WWWDSTAR import AbstractToGraph,IDstarLite

def fctCAST(obj,plt,algo):
    from PIL import ImageTk
    import time
    from psetSolutions import heuristicwithcurrents, costwithcurrents
    
    zoom = 2.5
    
    # Send AUV on exploration journey
    ## fix its start point
    xCO = obj.AUV.origin[0]
    yCO = obj.AUV.origin[1]
    
    PlacesVisited = [(xCO,yCO)]
    
    ## initialize vehicle status
    finished = False
    collisionFree = True
    
    def get_rotated_object_coordinates(angle,xoffset,yoffset,coordinates):
        """
        angle - rotate to angle (NOT relative!) 
        ...need to rotate coordinates back first!
        """
        RotatedCoordinates = []
        cangle = cmath.exp( radians(angle)*1j) # angle in degrees
    
        center = complex(xoffset, yoffset)
        for x, y in coordinates:
            v = cangle * (complex(x, y) - center) + center
            RotatedCoordinates.append( (v.real,v.imag) )
        return RotatedCoordinates
    
    def TranslatePath(path):
        executbl = []
        for index,_ in enumerate(range(len(path)-1)):
            distx = path[index + 1]['position'][0] - path[index]['position'][0]
            disty = path[index + 1]['position'][1] - path[index]['position'][1]

            # Get velocity of AUV from thrust vector and current
            speedx = path[index]['V_AUV'][0] + obj.Environment.CurrentField_x[ path[index]['position'][0] ][ path[index]['position'][1] ]
            speedy = path[index]['V_AUV'][1] + obj.Environment.CurrentField_y[ path[index]['position'][0] ][ path[index]['position'][1] ]
            
            # Get orientation of AUV from thrust vector V_AUV
            orientation = np.angle( complex(path[index]['V_AUV'][0],path[index]['V_AUV'][1]) , deg=True)
            executbl.append( (distx,disty,speedx,speedy,orientation) )
            
        return executbl
    
    def GeometryToTkinter(AUVCoordinates):
        AUVCoordinates_tkinter = []
        for pt in AUVCoordinates:
            xtkinter = (1148 - 311)/100 * pt[0] + 311
            ytkinter = (1025 - 186)/100 * (100- pt[1]) + 186
            AUVCoordinates_tkinter.append(xtkinter)
            AUVCoordinates_tkinter.append(ytkinter)
        return AUVCoordinates_tkinter
    
    def LengthsToTkinter(x,y):
        xtkinter = (1148 - 311)/100 * x
        ytkinter = (1025 - 186)/100 * y
        return xtkinter,ytkinter
    
    def sense(xCO,yCO):
        """Our sense function here checks for obstacles in the model, not exactly 
        in the picture (what we might do to make it more realistic)
        > act on the model"""
        
        # check each of the Dgrid points in the circle for obstacles and return coordinates
        PathBlocked = False
        coords = []
        checkrange = [(-1,+4),(0,+4),(1,+4),\
         (-2,+3),(-1,+3),(0,+3),(1,+3),(2,+3),\
         (-3,+2),(-2,+2),(-1,+2),(0,+2),(1,+2),(2,+2),(3,+2),\
         (-4,+1),(-3,+1),(-2,+1),(-1,+1),(0,+1),(1,+1),(2,+1),(3,+1),(4,+1),\
         (-4,+0),(-3,+0),(-2,+0),(-1,+0),(1,+0),(2,+0),(3,+0),(4,+0),\
         (-4,-1),(-3,-1),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),(3,-1),(4,-1),\
         (-3,-2),(-2,-2),(-1,-2),(0,-2),(1,-2),(2,-2),(3,-2),\
         (-2,-3),(-1,-3),(0,-3),(1,-3),(2,-3),\
         (-1,-4),(0,-4),(1,-4)]
        for checkpoint in checkrange:
            if (xCO + checkpoint[0], yCO + checkpoint[1]) in obj.Environment.UnknownElements:
                coords.append((xCO + checkpoint[0], yCO + checkpoint[1]))
                # Check if the newly detected obstacle is in path (access path given!)
                path = [(obj.AUV.path[z]['position'][0],obj.AUV.path[z]['position'][1]) for z in range(len(obj.AUV.path))]
                if (xCO + checkpoint[0], yCO + checkpoint[1]) in path:
                    PathBlocked = True
        return PathBlocked,coords
    
    def IndicatePath():
        """ Indicate replanned, currently intended path """
        TwoDpath = [(waypoint['position'][0],waypoint['position'][1]) for waypoint in obj.AUV.path]
        TwoDpathTkinter = [GeometryToTkinter([waypoint]) for waypoint in TwoDpath]
        for index in range(len(TwoDpath)-1):
            from_x  = TwoDpathTkinter[index][0]
            from_y  = TwoDpathTkinter[index][1]
            to_x    = TwoDpathTkinter[index+1][0]
            to_y    = TwoDpathTkinter[index+1][1]
            canvas.create_line(from_x,from_y,to_x,to_y, fill="blue")
    
    def VisualizePathBehind():
        for place in PlacesVisited:
            x = GeometryToTkinter([place])[0]
            y = GeometryToTkinter([place])[1]
            canvas.create_oval(x+2, y+2, x-2, y-2, fill='green')
            
    def UpdateEnvironment():
        file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png" # initialize to orientation straight (90 degrees)
        Env_org = Image.open(file)
        EnvWidth_org, EnvHeight_org = Env_org.size
        factor = zoom
        EnvWidth = int(EnvWidth_org * factor)
        EnvHeight = int(EnvHeight_org * factor)
        Env_zoomed = ImageTk.PhotoImage(Env_org.resize((EnvWidth,EnvHeight), Image.ANTIALIAS))
        canvas.create_image(0, 0, image=Env_zoomed, anchor = NW)
        
    def UpdateAUV():
        file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\AUV0.png" # initialize to orientation straight (90 degrees)
        img_org = Image.open(file)
        width_org, height_org = img_org.size
        factor = 1.65*0.07*zoom
        width = int(width_org * factor)
        height = int(height_org * factor)
        img_zoomed = img_org.resize((width,height), Image.ANTIALIAS)
        auvIm = ImageTk.PhotoImage(img_zoomed.rotate(90)) # we put it into the water in this orientation
        canvas.create_image(*GeometryToTkinter([(xCO,yCO)]), image = auvIm, anchor = CENTER)
    
    exe = TranslatePath(obj.AUV.path)
    
    # create the canvas simulation environment
    root = Tk()
    canvas = Canvas(width = zoom*900, height = zoom*500, bg = 'white')
    canvas.pack(expand = NO, fill = BOTH)
    
    # Zoom on environment
    file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png" # initialize to orientation straight (90 degrees)
    Env_org = Image.open(file)
    EnvWidth_org, EnvHeight_org = Env_org.size
    factor = zoom
    EnvWidth = int(EnvWidth_org * factor)
    EnvHeight = int(EnvHeight_org * factor)
    Env_zoomed = ImageTk.PhotoImage(Env_org.resize((EnvWidth,EnvHeight), Image.ANTIALIAS))
    canvas.create_image(0, 0, image=Env_zoomed, anchor = NW)
    
    # Indicate replanned path
    IndicatePath()
    
    # Zoom on AUV
    file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\AUV0.png" # initialize to orientation straight (90 degrees)
    img_org = Image.open(file)
    width_org, height_org = img_org.size
    factor = 1.65*0.07*zoom
    width = int(width_org * factor)
    height = int(height_org * factor)
    img_zoomed = img_org.resize((width,height), Image.ANTIALIAS)
    auvIm = ImageTk.PhotoImage(img_zoomed.rotate(90)) # we put it into the water in this orientation
    canvas.create_image(*GeometryToTkinter([(xCO,yCO)]), image = auvIm, anchor = CENTER)
    
    # pack the canvas
    canvas.pack()
        
    # move and explore
    i = 0
    while not finished and collisionFree:
        
        # Check for collisions 
        PathBlocked,coords = sense(xCO,yCO)
        
        # update model of the environment
        #TODO: This whole thing should be part of the "sense" function
        new_knowledge = False
        if algo == "DstarLite": obj.AlgorithmModel['DstarLite']['graph'].graph['updates'] = {'risk': {},'current': {}}
        for coord in coords:
            if obj.Environment.RiskField[coord[0]][coord[1]] != 1.0:
                new_knowledge = True
                if algo == "DstarLite": obj.AlgorithmModel['DstarLite']['graph'].graph['updates']['risk'][coord] = 1.0 
            obj.Environment.RiskField[coord[0]][coord[1]] = 1.0
        
        ### Show updated state of knowledge if new knowledge
        if new_knowledge == True:
            VIS1 = VISUALIZATION(obj.AUV,obj.Environment)
            VIS1.ShowReef()
            VIS1.ShowRisk()
            plt.savefig('foo.png')
            plt.close()
            file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png" # initialize to orientation straight (90 degrees)
            Env_org = Image.open(file)
            EnvWidth_org, EnvHeight_org = Env_org.size
            factor = zoom
            EnvWidth = int(EnvWidth_org * factor)
            EnvHeight = int(EnvHeight_org * factor)
            Env_zoomed = ImageTk.PhotoImage(Env_org.resize((EnvWidth,EnvHeight), Image.ANTIALIAS))
            canvas.create_image(0, 0, image=Env_zoomed, anchor = NW)
            # indicate originally planned path
            IndicatePath()
            
            # indicate AUV at position
            canvas.create_image(*GeometryToTkinter([(xCO,yCO)]), image = auvIm, anchor = CENTER)#xCO,yCO 
            
            ### Mark the path that the vehicle has traversed already
            VisualizePathBehind()
            ### Mark the path that the vehicle has traversed already
            
            canvas.update()      
        ### Show updated state of knowledge if new knowledge
        
        # Check if vehicle is at goal
        if xCO == obj.AUV.goal[0] and yCO == obj.AUV.goal[1]:
            finished = True
            
        # At this point we have CHECKED FOR GOAL AND COLLISIONS
         
        if finished != True and PathBlocked == True: # then replan  
            # create and save image of environment
            VIS1 = VISUALIZATION(obj.AUV,obj.Environment)
            VIS1.ShowReef()
            VIS1.ShowRisk()
            plt.savefig('foo.png')
            plt.close()
            
            # insert image in tkinter
            image = ImageTk.PhotoImage(file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png")
            canvas.create_image(0, 0, image = image, anchor = NW)
            # Zoom on environment
            file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png" # initialize to orientation straight (90 degrees)
            Env_org = Image.open(file)
            EnvWidth_org, EnvHeight_org = Env_org.size
            factor = zoom
            EnvWidth = int(EnvWidth_org * factor)
            EnvHeight = int(EnvHeight_org * factor)
            Env_zoomed = ImageTk.PhotoImage(Env_org.resize((EnvWidth,EnvHeight), Image.ANTIALIAS))
            canvas.create_image(0, 0, image=Env_zoomed, anchor = NW)
            
            # Call path planner (either A* or D* Lite)
            # Interface with A*
            #obj.AUV.origin = (xCO,yCO,-9.5)
            #obj.AUV.PlanPath(alg = algo, cost = costwithcurrents, heuristic = heuristicwithcurrents, env = obj.Environment, vis=None,  alpha=1)
            # Interface with A*
            
            # Interface with D*lite # TODO: This should be the generic interface later (see "Main")
            obj.AUV.path,_,obj.AlgorithmModel['DstarLite']['graph'] = IDstarLite(obj.AlgorithmModel['DstarLite']['graph'],start=(xCO,yCO), goal=(obj.AUV.goal[0],obj.AUV.goal[1]),obj=VIS1)
            # Interface with D*lite
            
            # Extract executable from path
            exe = TranslatePath(obj.AUV.path)
            
            # Indicate replanned path
            IndicatePath()
            
            ### Update AUV 
            orientation = exe[0][4]
            auvIm = ImageTk.PhotoImage(img_zoomed.rotate(orientation))
            canvas.create_image(*GeometryToTkinter([(xCO,yCO)]), image = auvIm, anchor = CENTER)#xCO,yCO
            ### Update AUV
            
            # Mark the path that the vehicle has traversed already
            VisualizePathBehind()
            
            canvas.update()
            
            # initialize for next calculation step
            i = 0
            PathBlocked = False
        
        if finished != True and PathBlocked == False: # then take a step
            xProgress   = exe[i][0]
            yProgress   = exe[i][1]
            speedx  = exe[i][2]
            speedy  = exe[i][3]
            orientation = exe[i][4]
            t = (xProgress**2 + yProgress**2)**0.5/(speedx**2 + speedy**2)**0.5
            
            # pause to make simulation real-time
            time.sleep(t)
            
            # new coords of vehicle
            xCO = xCO + xProgress
            yCO = yCO + yProgress
            PlacesVisited.append((xCO,yCO))
            
            ### Update AUV
            auvIm = ImageTk.PhotoImage(img_zoomed.rotate(orientation))
            canvas.create_image(*GeometryToTkinter([(xCO,yCO)]), image = auvIm, anchor = CENTER)#xCO,yCO
            ### Update AUV
            
            ### Mark the path that the vehicle has traversed already
            VisualizePathBehind()
            ### Mark the path that the vehicle has traversed already
            
            canvas.update()      
        
        # show the updated map with AUV position
        canvas.update()
        i += 1
    
    # start the animation
    #root.destroy()
    root.mainloop()