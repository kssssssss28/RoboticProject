import os
import time
import pybullet as p
import gym
from gym import spaces
import numpy as np
import random
import pybullet_data
import json
from kuka import Robot
from utils import euclidean_distance
class GARBAGE():

    def __init__(self, number):
        garbageMap = ["red","blue","green"]
        self.threePath = [0.8, 0.8, 0.8]
        #elf.threePath = [0.3, 0.55, 0.8]
        with open('../simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))

        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number
        count = 0
        for i in garbageInfo:  
            count = count + 4
            type = garbageMap[int(i["obj_id"])]
            # r = random.uniform(0, radius)
            path = "../simulation/urdf/" + str(type) + "Box.urdf"
            boxInfo = dict()
            boxInfo["name"]= str(type) + "Box"
            boxInfo["path"]= path
            boxInfo["type"]= self.color2int(str(type))
            boxInfo["startOri"] = startOrientation
            boxInfo["startPos"] = [0, 0,  0.52]
            boxInfo["boxId"] =  count
            self.garbageData.append(boxInfo)

        self.garbageData = list(self.garbageData)

    def generateGarbage(self):
        rd = random.randint(0, self.number)
        garbage = self.garbageData[rd]
        path = garbage["path"]
        rdPath = random.randint(0,2)
        rdPosition = [self.threePath[rdPath],0,.4]
        garbage["startPos"] = rdPosition
        startPos = garbage["startPos"]
        startOri = garbage["startOri"]
        boxId = p.loadURDF(path, startPos, startOri)
        p.changeDynamics(boxId,-1,mass = 5)
        self.garbageData[rd]["boxId"] = boxId
        # print('---'*10)
        # print('The type of garbage is: ', self.garbageData[rd]["type"])
        # print('---'*10)
    
    def deleteGarbage(self):
        for garbage in self.garbageData:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None
    
    def color2int(self, type):
        if type == 'red':
            return 0
        elif type == 'blue':
            return 1
        else:
            return 2

class GarbageSortingEnv(gym.Env):
    def __init__(self, gui=True, num_robots=1, num_garbage=1, camera_pos=[1, .5, 0], camera_distance=4, conveyor_speed=1, garbage_delay=4000):
        super().__init__()

        # Initialize environment parameters
        self.gui = gui
        self.num_robots = num_robots
        self.num_garbage = num_garbage
        self.camera_pos = camera_pos
        self.camera_distance = camera_distance
        self.conveyor_speed = conveyor_speed
        self.garbage_delay = garbage_delay
        self.move_right = 0.2  
        self.steps = 0
        self.total_steps = 1500

        self.garbage = GARBAGE(self.num_garbage)
        # self.reset()
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=.3, high=1.2, shape=(3,), dtype=np.float32)
        # robot,           gripper position garbage pos gripper status distance(g and d) distance(g and g) 
        observation_space_shape =  8
        
        self.observation_space = spaces.Box(low=0, high=1, shape=(observation_space_shape,), dtype=np.float32)

        p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity to simulate real-world
        p.setGravity(0, 0, -9.81)

        # Set camera
        p.resetDebugVisualizerCamera(
            cameraDistance=self.camera_distance,
            cameraYaw=0,
            cameraPitch=269,
            cameraTargetPosition=self.camera_pos
        )

        # Load the environment models
        self.load_models()

    def load_models(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        ground_id = p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf")

        x_color = [1, 0, 0]  
        y_color = [0, 1, 0]  
        z_color = [0, 0, 1] 

        p.addUserDebugLine([0, 0, 2], [5, 0, 2], x_color, 5, 20000)
        p.addUserDebugLine([0, 0, 2], [0, 5, 2], y_color, 5, 20000)
        p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)


        # Not necessary to initialize any garbage here, the reset includes garbage


        # ------------------------- initialize robotic arm ------------------------------
        self.kuka = Robot()
        self.robot_arm_ids = self.kuka.get_robot_info("id")
        # ------------------------- robotic arm ------------------------------
   

        # ------------------------- initialize conveyor belt ------------------------------
        conveyor_pos = [0.56, 0, 0.1]
        conveyor_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.conveyor_id = p.loadURDF("../simulation/urdf/block.urdf", conveyor_pos, conveyor_ori)
        conveyor_speed = self.conveyor_speed   
        conveyor_joint_index = 0 
        p.setJointMotorControl2(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)
        # ------------------------- conveyor belt ------------------------------
        constraint_id = p.createConstraint(ground_id, -1, self.conveyor_id, -1, p.JOINT_FIXED, [0, 0, 0], [0.56, 0, 0.1], [0, 0, 0])

    def step(self, action):

        
        
        # Step simulation
        p.stepSimulation()

        # Move the conveyor belt
        conveyor_id = self.conveyor_id
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.setJointMotorControl2(conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)

        # Get observation
        observation = self.get_observation()
        
        effector_position = [observation[3], observation[4], observation[5]]
        garbage_positions = [observation[0], observation[1], observation[2]]
        
        ggDistance = observation[6]
        gdDistance = observation[7]
        print("======================",gdDistance)
        if ggDistance <= 0.2:
            self.kuka.grab(0,ggDistance,self.garbage.garbageData[0]["boxId"])
            self.kuka.move2Area()    
            if gdDistance <= 0.2:
                self.kuka.release()
  
        else: 
            # Apply action to the robotic arm
            targetPosition = [action[0], action[1], action[2]]
            p.addUserDebugLine([0,0,0], targetPosition, [1,0,0], 0.5, 3)
            self.kuka.moveArm(self.kuka.get_robot_info("id"),9999,targetPosition)
            

           
        
        self.steps += 1

        # Calculate reward
        reward = self.calculate_reward(observation)

        # Check if the episode is done
        done = self.is_done(observation) or self.steps >= self.total_steps

        return observation, reward, done, {}



    def reset(self):
        # print('Reset!!!!!!')
        debug = False

        self.steps = 0

        # Reset robot arm joint positions
        joint_positions = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # UR5 example joint positions
        for i, position in enumerate(joint_positions):
            p.resetJointState(self.robot_arm_ids, i, position)

        # Remove previous garbage objects
        for garbage in self.garbage.garbageData:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None

        # Reset and re-generate garbage objects
        self.garbage = GARBAGE(self.num_garbage)
        self.garbage.generateGarbage()

        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']
        # print('-'*30)
        # print('Name is:', name_)
        # print('Type is: ',type_)
        # print('-'*30)

        # Reset conveyor belt
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.resetJointState(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL,targetVelocity=conveyor_speed)

        # Get observation
        observation = self.get_observation()
        effector_position = [observation[3], observation[4], observation[5]]
        garbage_positions = [observation[0], observation[1], observation[2]]
        ggDistance = observation[6]
        gdDistance = observation[7]

        if debug: 
            print("Effector position:\n", effector_position)
            print("Garbage positions:\n", garbage_positions)
            print("Distance:\n", ggDistance,gdDistance)

        return observation

    
    def get_observation(self):
        debug = True    

        # Get end-effector position
        effector_position = p.getBasePositionAndOrientation(2)[0]

                
        garbage_positions = p.getBasePositionAndOrientation(self.garbage.garbageData[0]["boxId"])[0]

        # Get distance between garbage and gripper
        garbageDistanceGripper = euclidean_distance(effector_position, garbage_positions)

        # Get distance between garbage and destination
        garbageDistanceDes = euclidean_distance([1.2,.8,0.8], garbage_positions)
       
        # color = 
        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']
    

        
        
        # if debug:
        #     print('-'*30)
        #     print('closet garbage position is:')
        #     print(garbage_positions)
        #     print('ee position is:')
        #     print(effector_position)
        #     print('gg distance is:')
        #     print(garbageDistanceGripper)
        #     print('gd distance is:')
        #     print(garbageDistanceDes)

        #     print('-'*30)
        
        garbageDistanceGripper = np.array([garbageDistanceGripper])
        garbageDistanceDes = np.array([garbageDistanceDes])

        # Concatenate observations
        observation = np.concatenate((garbage_positions, effector_position, garbageDistanceGripper, garbageDistanceDes))
        
        return observation


    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

    def is_done(self, observation):
        garbage_positions =[observation[0], observation[1], observation[2]]
        gdDistance = observation[-1]
        # Check if garbage has fallen off the conveyor
        if garbage_positions[2] < 0.3:
            return True
        
        # Check if the robotic arm has successfully moved the garbage to its correct line
        success = False
        
        if gdDistance <= 0.2:
            success = True
        
        if self.steps >= 1500:
            return True

        return success

    def calculate_reward(self, observation):
        
        gdDistance = observation[-1]

        reward = 0

        if gdDistance <= 0.1:
            reward = reward + 0.1
        if gdDistance <= 0.05:
            reward = reward + 0.5
        if gdDistance <= 0.01:
            reward = reward + 1
        if gdDistance >= 0.1:
            reward = reward - 0.2
        if gdDistance >= 0.5:
            reward = reward - 0.5
        

        return reward
    


'''
For debug use

Before the garbages are placed in the conveyor belt, we assume that there already has a machine that 
separate all messy garbages into three lines:

Red:   0
Blue:  1
Green: 2

         0.8        0.55        0.3
         Red        Green       Blue

    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |
    |           |           |           |


right:
position and oritation is:
        stay                  up to down          lines
[((0.3004484709079834, -1.2761461061828077, 0.35997787684410787), 
        zero                   zero                   zero                      one
(-3.201844940525437e-08, -2.010115023422663e-05, -1.789480748153126e-05, 0.9999999996378592))]

garbage position:

(左右, 上下, 里外)

'''


