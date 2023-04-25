import math
import pybullet as p
from utils import euclidean_distance

class Robot():
    def __init__(self):
        robotId = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000, -0.200000, 0.600000, 0.000000, 0.000000, 0.000000, 1.000000)
        table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.2, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
        posCube = p.loadURDF("cube.urdf", basePosition=[0, -0.2, 0.65], globalScaling=0.0001)
        p.createConstraint(robotId, 6, posCube, -1, p.JOINT_FIXED, 
                                        [0, 0, 0], [0, 0, 0.05], [0, 0, 0], 
                                        [0,0,0])
        print("=====init=====",p.getBasePositionAndOrientation(3)[0])
        for jointIndex in range(p.getNumJoints(robotId)):
            p.resetJointState(robotId, jointIndex, jointPositions[jointIndex])
            p.setJointMotorControl2(robotId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
                # 获取关节状态
        robots = list()
        robot = dict()
        robot["id"]= robotId
        robot["tempConstraint"] = None
        numJoints = p.getNumJoints(robotId)
        jointInfo = [p.getJointInfo(robotId, i) for i in range(numJoints)]
        availableJoint = []
        for joint in jointInfo:
            item = dict()
            if joint[2] != 4:
                item["id"] = joint[0]
                item["name"] = joint[1]
                availableJoint.append(item)
        robot["availableJoint"] = availableJoint
        robot["posCube"] = posCube
        robots.append(robot)
        self.robots = robots
    
            
            
    def moveArm(self, id, cubeId, pos, ori=[0,0,0]):
        if cubeId <= 999:
            pos = p.getBasePositionAndOrientation(cubeId)[0]        
        robot = self.robots[0]
        id= robot["id"]
        joint_poses = p.calculateInverseKinematics(id,
                                            6,
                                            pos,
                                            targetOrientation=[],
                                            )
        p.addUserDebugLine([0, 0, 0], pos, [1, 0, 0], 5, 5)
        for i in range(len(joint_poses)):
            p.setJointMotorControl2(id,
                                    robot["availableJoint"][i]["id"],
                                    p.POSITION_CONTROL,
                                    joint_poses[i],                                                    
                                    )

    def get_robot_info(self):
        return self.robots[0]
    
    def grab(self, cubeId):
        robot = self.robots[0]
        kuka_id = robot['id']
        cubePos = p.getBasePositionAndOrientation(cubeId)[0]

        if robot["tempConstraint"] is not None:
            p.removeConstraint(robot["tempConstraint"])

        self.moveArm(0,cubeId,cubePos)

        endPos = p.getBasePositionAndOrientation(3)[0]
        distance = euclidean_distance(cubePos, endPos)
        if distance <= 0.1:
            kuka_cid = p.createConstraint(kuka_id, 6, 
                                        cubeId, -1, p.JOINT_FIXED, 
                                        [0, 0, 0], [0, 0, 0.05], [0, 0, 0], 
                                        [0,0,0])
            robot["tempConstraint"] = kuka_cid
            self.move2Area()
        else: 
            print(" too far ")

    def release(self):
        robot = self.robots[0]
        if robot["tempConstraint"] is not None:
            p.removeConstraint(robot["tempConstraint"])
            robot["tempConstraint"] = None
        print(self.robots[0])

       
        

    
    def move2Area(self, area=0):
        robot = self.robots[0]
        robotId = robot["id"]
        if area == 0:
            self.moveArm(0,9999,[2.3,-0.2,1.3])