from tkinter import *
from PIL import ImageTk

canvas = Canvas(width = 500, height = 500, bg = 'blue')
canvas.pack(expand = NO, fill = BOTH)

# create image of environment
image = ImageTk.PhotoImage(file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\foo.png")
canvas.create_image(0, 0, image = image, anchor = NW)

# scale image
# scale_w = new_width/old_width
# scale_h = new_height/old_height
# photoImg.zoom(scale_w, scale_h)

def tkinter_coordinates(AUVCoordinates):
    AUVCoordinates_tkinter = []
    for pt in AUVCoordinates:
        xtkinter = (459 - 124)/100 * pt[0] + 124
        ytkinter = (410 - 75)/100 * (100- pt[1]) + 75
        AUVCoordinates_tkinter.append(xtkinter)
        AUVCoordinates_tkinter.append(ytkinter)
    return AUVCoordinates_tkinter

# draw AUV with AUV vehicle size data
xCO = 10
yCO = 20
nose    = [xCO + 0.   , yCO + 2.0]
noseL   = [xCO - 0.5 , yCO + 1.5]
noseR   = [xCO + 0.5 , yCO + 1.5]
tailL   = [xCO - 0.5 , yCO - 1.0]
tailR   = [xCO + 0.5 , yCO - 1.0]
AUVgeometry = [nose,noseR,tailR,tailL,noseL]
canvas.create_polygon(*tkinter_coordinates(AUVgeometry),outline='black',fill='yellow',width=0)

canvas.pack()

mainloop()