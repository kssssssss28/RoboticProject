
import pybullet as p
import json
import random
import numpy as np


# 圆心坐标
center = (1, -1)
# 圆半径
radius = 1.5
# Joints information includes index and name
# [[1, b'shoulder_pan_joint'], 
# [2, b'shoulder_lift_joint'], 
# [3, b'elbow_joint'], 
# [4, b'wrist_1_joint'], 
# [5, b'wrist_2_joint'], 
# [6, b'wrist_3_joint'], 
# [9, b'finger_joint'], 
# [11, b'left_inner_finger_joint'], 
# [13, b'left_inner_knuckle_joint'],  
# [14, b'right_outer_knuckle_joint'], 
# [16, b'right_inner_finger_joint'], 
# [18, b'right_inner_knuckle_joint']]
class GARBAGE():

    def __init__(self, number):
        garbageMap = ["red","blue","green"]
        self.threePath = [0.3, 0.55, 0.8]
        
        with open('../simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))

        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number

        for i in garbageInfo:  
            type = garbageMap[int(i["obj_id"])]
            r = random.uniform(0, radius)
            path = "../simulation/urdf/" + str(type) + "Box.urdf"
            boxInfo = dict()
            boxInfo["name"]= str(type) + "Box"
            boxInfo["path"]= path
            boxInfo["type"]= self.color2int(str(type))
            boxInfo["startOri"] = startOrientation
            boxInfo["startPos"] = [0, 0,  0.52]
            boxInfo["boxId"] = None
            self.garbageData.append(boxInfo)

        self.garbageData = list(self.garbageData)

    def generateGarbage(self):
        rd = random.randint(0, self.number)
        garbage = self.garbageData[rd]
        path = garbage["path"]
        rdPath = random.randint(0,2)
        rdPosition = [self.threePath[rdPath],-3,.4]
        garbage["startPos"] = rdPosition
        startPos = garbage["startPos"]
        startOri = garbage["startOri"]
        boxId = p.loadURDF(path, startPos, startOri)
        p.changeDynamics(boxId,-1,mass = 5)
        self.garbageData[rd]["boxId"] = boxId
        # print('---'*10)
        # print('The type of garbage is: ', self.garbageData[rd]["type"])
        # print('---'*10)
    
    def color2int(self, type):
        if type == 'red':
            return 0
        elif type == 'blue':
            return 1
        else:
            return 2


class CONVEYOR():
    def __init__(self,speed):

        conveyor_pos = [0.56, 0, 0.1]
        conveyor_ori = p.getQuaternionFromEuler([0, 0, 0])
        conveyor_id = p.loadURDF("../simulation/urdf/block.urdf", conveyor_pos, conveyor_ori)
        

        conveyor_speed = speed  # m/s
        conveyor_joint_index = 0 
        p.setJointMotorControl2(conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)


class UR5():
    def __init__(self, number):
        self.robots= list()
        for robot in range(number):
            if robot % 2 == 0:
                startPos = [0,robot-0.8,0.47]
                startOrientation = p.getQuaternionFromEuler([0, 0, 0]) # Rotate 180 degrees around z-axis
            else:
                startPos = [1.2,robot-0.8,0.47]
                startOrientation = p.getQuaternionFromEuler([0, 0, 3.15])

            robot_id = p.loadURDF('../urdf/ur5_robotiq_85.urdf', startPos, startOrientation)
            availableJoints = [i for i in range(p.getNumJoints(robot_id)) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]


            jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
            numJoints = p.getNumJoints(robot_id)

            robot = dict()
            robot["id"] = robot_id
            robot["joints"] = []
            robot["movableJoints"] = []
            for i in range(numJoints):
                
                info = p.getJointInfo(robot_id,i)
                item = dict()
                item["jointID"] = info[0]
                item['jointName'] = info[1].decode('utf-8')
                item['jointType'] = jointTypeList[info[2]] 
                item['jointLowerLimit'] = info[8]
                item['jointUpperLimit'] = info[9]
                item['jointMaxForce'] = info[10]
                item['jointMaxVelocity'] = info[11]
                robot["joints"].append(item)

                if i in availableJoints:
                    robot["movableJoints"].append([info[0], info[1].decode('utf-8')])

            self.robots.append(robot)

    def moveArm(self, id, pos, ori):
        robot = self.robots[id - 1]
        joint_poses = p.calculateInverseKinematics(id,
                                           18,
                                           pos,
                                           targetOrientation=[],
                                           )

        for i in range(len(joint_poses)):
            p.setJointMotorControl2(id,
                                    robot["movableJoints"][i][0],
                                    p.POSITION_CONTROL,
                                    joint_poses[i],                       
                                    )

    def closeGripper(self,index):
        p.setJointMotorControl2(index,11,p.VELOCITY_CONTROL,30,50)
        p.setJointMotorControl2(index,16,p.VELOCITY_CONTROL,30,50)


    def openGripper(self,index):
        p.setJointMotorControl2(index,11,p.VELOCITY_CONTROL,30,-50)
        p.setJointMotorControl2(index,16,p.VELOCITY_CONTROL,30,-50)
