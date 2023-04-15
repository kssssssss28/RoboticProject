import pybullet as p
import time
import pybullet_data
from robot import GARBAGE
import pybullet as p
import numpy as np
from PIL import Image
import random
def saveImg():
        # 将图像数据转换为 RGB 图像

        # 获取相机图像
    img_arr = p.getCameraImage(width=480, height=320)

    rgb_arr = np.array(img_arr[2])[:,:,[2,1,0]] # BGR to RGB
    img = Image.fromarray(rgb_arr)
    # 保存图像
    img.save("image.png")

def initWorld(GUI):
    physicsClient = p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # load ground
    planeId = p.loadURDF("plane.urdf")


    # 定义 x，y，z 坐标轴的颜色
    x_color = [1, 0, 0] # 红色
    y_color = [0, 1, 0] # 绿色
    z_color = [0, 0, 1] 


    # 在 (0,0,0.5) 处画 x，y，z 坐标轴
    p.addUserDebugLine([0, 0, 3], [5, 0, 3], x_color, 5, 20000)
    p.addUserDebugLine([0, 0, 3], [0, 5, 3], y_color, 5, 20000)
    p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)



    # 设置相机位置和方向
    p.resetDebugVisualizerCamera(
        cameraDistance=4,
        cameraYaw=0,
        cameraPitch=269,
        cameraTargetPosition=[1.2, -.7, 0]
    )




    

def starSimulation():

    box = GARBAGE(20)
    count = 3500
    while (True): 
        print(count)
        rd = random.randint(0, 100)

        if count >= 7000:
            count = 0
            box.generateGarbage()
            saveImg()

        p.stepSimulation() 
        count = count + rd
        time.sleep(1./240.) # 暂停一下，让仿真按照正常时间流逝       


def generateGarbage():
        box = GARBAGE()