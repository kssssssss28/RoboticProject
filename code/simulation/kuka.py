import math
import pybullet as p
from utils import euclidean_distance

class Robot():
    def __init__(self):
        # robotId = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 
        #                      .700000, 1.0000, .5500000, 0.000000, 0.000000, 1.00000, 1.000000)
        robotId = p.loadURDF('../urdf/ur5_robotiq_85.urdf', .6300000, 1.3000, .900000, 0.000000, 0.000000, -4.0000, 1.000000)
        self.wait = 0
        # 获取机器人的当前姿势
        pos, orn = p.getBasePositionAndOrientation(robotId)

        # 计算90度旋转所需的四元数
        rot_quat = p.getQuaternionFromEuler([0, 0, 2])

        # 将机器人的朝向乘以所计算的四元数，以获得旋转后的新姿势
        new_orn = p.multiplyTransforms([0, 0, 0], rot_quat, [0, 0, 0], orn)[1]

        # 设置机器人的新姿势
        p.resetBasePositionAndOrientation(robotId, pos, new_orn)
        posCube = p.loadURDF("cube.urdf", basePosition=[0, -0.2, 0.65], globalScaling=0.000001)

        p.createConstraint(robotId, 16, posCube, -1, p.JOINT_FIXED, 
                                        [0, 0, 0], [0, 0, 0.05], [0, 0, 0], 
                                        [0,0,0])
        
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

    def getCurrentPos(self):
        return p.getBasePositionAndOrientation(2)[0]
        
    def moveArm(self, id, cubeId, pos, ori=[0,0,0]):
        if cubeId <= 999:
            pos = p.getBasePositionAndOrientation(cubeId)[0]       
        robot = self.robots[0]
        id= robot["id"]
        joint_poses = p.calculateInverseKinematics(id,
                                            17,
                                            pos,
                                            targetOrientation=[],
                                            )
        for i in range(len(joint_poses)):
            p.setJointMotorControl2(id,
                                    robot["availableJoint"][i]["id"],
                                    p.POSITION_CONTROL,
                                    joint_poses[i],                                                    
                                    )
            
            
            
    def move2Area(self, index=0, area=0):
        self.moveArm(0,9999,[.6700000, 1.0000, 1.2])
        if area == 0 :
            pos = [1.2,.8,0.8]
            self.moveArm(self.get_robot_info("id"),9999,pos)
        elif area == 1:
            pos = [0,.8,0.8]
            self.moveArm(self.get_robot_info("id"),9999,pos)

    def get_robot_info(self, type):
        robot = self.robots[0]
        if type == "gripper":
            if robot["tempConstraint"] == None:
                return 0
            else:
                return 1
        return robot[type]
    
    def getGripperStatus(self):
        return self.robots[0]["tempConstraint"]
    
    def grab(self, cubeId):
        robot = self.robots[0]
        kuka_id = self.get_robot_info("id")
        kuka_cid = p.createConstraint(kuka_id, 15, 
                                        cubeId, -1, p.JOINT_FIXED, 
                                        [0, 0, 0], [0, 0, 0.05], [0, 0, 0], 
                                        [0,0,0])
        robot["tempConstraint"] = kuka_cid


    def release(self, wait):
        robot = self.robots[0]
        if robot["tempConstraint"] is not None:
            p.removeConstraint(robot["tempConstraint"])
            robot["tempConstraint"] = None


        
        
        
    
    def clearWait(self):
        self.wait = "Pause"

    
            