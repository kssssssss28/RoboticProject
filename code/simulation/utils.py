import pybullet as p
import time
import pybullet_data
from robot import GARBAGE
def initWorld(GUI):
    physicsClient = p.connect(p.GUI if GUI else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # load ground
    planeId = p.loadURDF("plane.urdf")

def starSimulation():
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1 / 240)

def generateGarbage():
        box = GARBAGE()