import pybullet as p
import time
import pybullet_data

import pybullet as p
import numpy as np
from PIL import Image
import random

def saveImg():
    img_arr = p.getCameraImage(width=480, height=320)
    rgb_arr = np.array(img_arr[2])[:,:,[2,1,0]] # BGR to RGB
    img = Image.fromarray(rgb_arr)
    img.save("../simulation/data/image.png")

def initWorld(GUI, cameraPos = [1, .5, 0], dis = 4):
    p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # load ground
    planeId = p.loadURDF("plane.urdf")


    x_color = [1, 0, 0]  
    y_color = [0, 1, 0]  
    z_color = [0, 0, 1] 

    p.addUserDebugLine([0, 0, 2], [5, 0, 2], x_color, 5, 20000)
    p.addUserDebugLine([0, 0, 2], [0, 5, 2], y_color, 5, 20000)
    p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)

    p.resetDebugVisualizerCamera(
        cameraDistance=dis,
        cameraYaw=0,
        cameraPitch=269,
        cameraTargetPosition=cameraPos
    )

# def starSimulation(totalGarbage, takeImg, delay = 4000):

#     box = GARBAGE(20)
#     count = 3500
#     total = 0
#     while (True): 

#         rd = random.randint(0, 50)


#         if count >= delay: 
#             count = 0
#             total = total + 1
#             if total < totalGarbage + 1:
#                 box.generateGarbage()
#                 if takeImg:
#                     saveImg()


#         if count >= delay + 200:
#             if takeImg:
#                 saveImg()



#         p.stepSimulation() 
#         count = count + rd
#         time.sleep(1./240.) 