from PIL import Image
import matplotlib.pyplot as plt

# get file and make it smaller
file = "C:\\Users\\micha\\OneDrive\\ECLIPSEWORKSPACE\\GrandChallenge\\AUV.png"
img_org = Image.open(file)
width_org, height_org = img_org.size
factor = 12
width = int(width_org * factor)
height = int(height_org * factor)
img_zoomed = img_org.resize((width,height), Image.ANTIALIAS)

plt.imshow(img_org)
plt.imshow(img_zoomed)

print('stop')