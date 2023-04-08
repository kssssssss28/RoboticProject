import pybullet as p
import time
import pybullet_data
from robot import GARBAGE
def initWorld(GUI):
    physicsClient = p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # load ground
    planeId = p.loadURDF("plane.urdf")

def starSimulation():
    while (True): # 模拟10000步
        p.stepSimulation() # 执行一步仿真
        time.sleep(1./240.) # 暂停一下，让仿真按照正常时间流逝       


def generateGarbage():
        box = GARBAGE()