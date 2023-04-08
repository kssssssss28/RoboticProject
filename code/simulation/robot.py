
import os
import time
import pybullet as p
import math
from collections import namedtuple
import json
import random
import math
import numpy as np
# 圆心坐标
center = (1, -1)
# 圆半径
radius = 1.5

class GARBAGE():

    def __init__(self, number):
        garbageMap = ["red","blue","green"]
        #generate init box
        with open('code/simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))
        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number
        for i in garbageInfo:  
            type = garbageMap[int(i["obj_id"])]
            r = random.uniform(0, radius)
            theta = random.uniform(0, 2 * math.pi)
            path = "code/simulation/urdf/" + str(type) + "box.urdf"
            boxInfo = dict()
            boxInfo["name"]= str(type) + "Box"
            boxInfo["path"]= path
            boxInfo["startOri"] = startOrientation
            boxInfo["startPos"] = [np.random.uniform(-0.7, 0.7), np.random.uniform(-20, -0.1),  0.13]
            self.garbageData.append(boxInfo)
            print(boxInfo)
        self.garbageData = list(self.garbageData)

    def generateGarbage(self):
        for i in range(self.number):
            x = int(random.uniform(0, len(self.garbageData) - 1))
            garbage = self.garbageData[x - 1]
            path = garbage["path"]
            startPos = garbage["startPos"]
            startOri = garbage["startOri"]
            boxId = p.loadURDF(path, startPos, startOri)
            p.changeDynamics(boxId,-1,mass = 5)
            self.garbageData[x - 1]["boxId"] = boxId

        





class UR5():
    def __init__(self):
        self.robotId = list()
        for robot in range(4):
            if robot%2 == 0 :
                startPos = [1, robot + 1.5, 0.4]
            if robot%2 != 0 :
                startPos = [-1, robot+ 1.5, 0.4]

            startOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
            #base = p.loadURDF('code/simulation/urdf/base.urdf', [0,0,0], startOrientation)
            self.id = p.loadURDF('code/simulation/urdf/ur5_robotiq_85.urdf', startPos, startOrientation)
            self.robotId.append(self.id)
            availableJoints = [i for i in range(p.getNumJoints(self.id)) if p.getJointInfo(self.id, i)[2] != p.JOINT_FIXED]
            #Available joint index and name
            joint4Pose = []
            for index in availableJoints:
                set = [p.getJointInfo(self.id, index)[0], p.getJointInfo(self.id, index)[1]]
                print(set)
                joint4Pose.append(set)
            

            #记录各个节点类型的列表
            jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
            #得到机器人的节点个数
            numJoints = p.getNumJoints(self.id)
            #属于提前创造存储节点信息的数据结构
            jointInfo = namedtuple("jointInfo",["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity"])
            #另一个数据结构
            joints = dict()
            for i in range(numJoints):
            #得到节点的信息
                info = p.getJointInfo(self.id,i)
                #将节点的各个信息提取出来
                jointID = info[0]
                jointName = info[1].decode('utf-8')
                jointType = jointTypeList[info[2]] 
                jointLowerLimit = info[8]
                jointUpperLimit = info[9]
                jointMaxForce = info[10]
                jointMaxVelocity = info[11]
                singleInfo = jointInfo(jointID,jointName,jointType,jointLowerLimit,jointUpperLimit,jointMaxForce,jointMaxVelocity)
                joints[singleInfo.name] = singleInfo


            p.setJointMotorControl2(self.id, 
                                4, 
                                p.POSITION_CONTROL, 
                                -2.5, 
                                targetVelocity=100, 
                                force=150, )
            p.setJointMotorControl2(self.id, 
                                3, 
                                p.POSITION_CONTROL, 
                                -2.3, 
                                targetVelocity=100, 
                                force=100, )

            p.setJointMotorControl2(self.id, 
                                5, 
                                p.POSITION_CONTROL, 
                                -2.5, 
                                targetVelocity=100, 
                                force=230, )


class CONVEYOR():
    def __init__(self):
        # 加载 URDF 模型
        conveyor_belt = p.loadURDF("code/simulation/urdf/block.urdf",[0,0,0])
        wall_start_orientation = p.getQuaternionFromEuler([0, 0, 0]) # 初始方向为没有旋转
        p.changeDynamics(conveyor_belt,0,lateralFriction = 100,spinningFriction = 100,rollingFriction = 50)
        p.setJointMotorControl2(conveyor_belt, 0, p.VELOCITY_CONTROL, targetVelocity=120, force=15000)


