
import pybullet as p
import math
from collections import namedtuple
import os 
import time 
class GARBAGE():
    def __init__(self):
        startPos = [-2, 0, 1]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.id = p.loadURDF('code/simulation/urdf/box.urdf', startPos, startOrientation)


class UR5():
    def __init__(self):
        startPos = [0, 0, 0.6]
        startOrientation = p.getQuaternionFromEuler([-math.pi/2, 0, 0])
        self.id = p.loadURDF('code/simulation/urdf/ur5_robotiq_85.urdf', startPos, startOrientation)
        # 获取机器人的所有关节信息
        availableJoints = [i for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] != p.JOINT_FIXED]
        joint4InitPose = []
        for index in availableJoints:
            set = [p.getJointInfo(self.id, index)[0], p.getJointInfo(self.id, index)[1]]
            joint4InitPose.append(set)

 


class CONVEYOR():
    def __init__(self):
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.id = p.loadURDF('code/simulation/urdf/conveyor.urdf', startPos, startOrientation)


