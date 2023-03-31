
import time
import pybullet as p
import math
from collections import namedtuple

class GARBAGE():
    def __init__(self):
        startPos = [-2, 0, 1]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.id = p.loadURDF('code/simulation/urdf/box.urdf', startPos, startOrientation)

class UR5():
    def __init__(self):
        startPos = [0, 0.5, 0.22]
        startOrientation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
        #base = p.loadURDF('code/simulation/urdf/base.urdf', [0,0,0], startOrientation)
        self.id = p.loadURDF('code/simulation/urdf/ur5_robotiq_85.urdf', startPos, startOrientation)
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
                            2, 
                            p.POSITION_CONTROL, 
                            -2.4, 
                            targetVelocity=10, 
                            force=15000, )
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
        for i in range(1000):
            p.stepSimulation()
            time.sleep(1/240)


class CONVEYOR():
    def __init__(self):
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.id = p.loadURDF('code/simulation/urdf/conveyor.urdf', startPos, startOrientation)


