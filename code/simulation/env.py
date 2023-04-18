import os
import time
import pybullet as p
import gym
from gym import spaces
import numpy as np
import random
import pybullet_data
import json

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
            # r = random.uniform(0, radius)
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
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_robots * 6,), dtype=np.float32)
        # robot, 
        observation_space_shape = 6*self.num_robots + 3 + 4 * self.num_garbage
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
        self.robot_arm_ids = p.loadURDF('../simulation/urdf/ur5_robotiq_85.urdf', [0 - self.move_right,-0.8,0.47], p.getQuaternionFromEuler([0, 0, 0]))
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
        
        # Apply action to the robotic arm
        for i, target_position in enumerate(action):
            p.setJointMotorControl2(self.robot_arm_ids, i, p.POSITION_CONTROL, targetPosition=target_position)

        # Step simulation
        p.stepSimulation()

        # Move the conveyor belt
        conveyor_id = self.conveyor_id
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.setJointMotorControl2(conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)

        # Get observation
        observation = self.get_observation()

        joint_positions = observation[:self.num_robots*6].reshape((self.num_robots, 6))
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]

        # print('--'*10)
        # print("Joint positions:\n", joint_positions)
        # print("Effector position:\n", effector_position)
        # print("Garbage positions:\n", garbage_positions)
        # print("Garbage type:\n", garbage_type)
        # print('--'*10)
        
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

        joint_positions = observation[:self.num_robots*6].reshape((self.num_robots, 6))
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]

        if debug: 
            print("Joint positions:\n", joint_positions)
            print("Effector position:\n", effector_position)
            print("Garbage positions:\n", garbage_positions)
            print("Garbage type:\n", garbage_type)

        return observation

    
    def get_observation(self):
        debug = False    

        # Get joint states
        joint_states = p.getJointStates(self.robot_arm_ids, range(6))  # Assuming UR5 with 6 joints
        joint_positions = np.array([state[0] for state in joint_states])

        # Get end-effector position
        effector_position, _ = p.getLinkState(self.robot_arm_ids, 5)[:2]  # Assuming UR5 with 6 joints

        # Get garbage objects' positions and orientations
        garbage_positions_orientations = [p.getBasePositionAndOrientation(garbage["boxId"]) for garbage in self.garbage.garbageData if garbage["boxId"] is not None]
        # color = 
        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']

        garbage_positions = np.array([pos for pos, _ in garbage_positions_orientations])

        if debug:
            print('-'*30)
            print('garbage position and orientation is:')
            print(garbage_positions_orientations)
            print('garbage position is:')
            print(garbage_positions)
            print('joint position is:')
            print(joint_positions)
            print('ee position is:')
            print(effector_position)
            print('-'*30)
        

        # Concatenate observations
        observation = np.concatenate((joint_positions, effector_position, garbage_positions.flatten(),[type_]))
        
        return observation


    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

    def is_done(self, observation):
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]
        
        # Check if garbage has fallen off the conveyor
        for pos in garbage_positions:
            if pos[2] < 0.3:  # Adjust threshold if needed
                return True

        # Check if the robotic arm has successfully moved the garbage to its correct line
        success = False
        for i, pos in enumerate(garbage_positions):
            if garbage_type == 0 and 0.25 < pos[0] < 0.35 and abs(effector_position[1] - pos[1]) > 0.1:
                success = True
            elif garbage_type == 1 and 0.5 < pos[0] < 0.6 and abs(effector_position[1] - pos[1]) > 0.1:
                success = True
            elif garbage_type == 2 and 0.75 < pos[0] < 0.85 and abs(effector_position[1] - pos[1]) > 0.1:
                success = True
        
        if self.steps >= 1500:
            return True

        return success

    def calculate_reward(self, observation):
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]

        reward = 0

        for i, pos in enumerate(garbage_positions):
            if garbage_type == 0:
                if 0.25 < pos[0] < 0.35:
                    reward += 1
                elif 0.5 < pos[0] < 0.6 or 0.75 < pos[0] < 0.85:
                    reward -= 1
            elif garbage_type == 1:
                if 0.5 < pos[0] < 0.6:
                    reward += 1
                elif 0.25 < pos[0] < 0.35 or 0.75 < pos[0] < 0.85:
                    reward -= 1
            elif garbage_type == 2:
                if 0.75 < pos[0] < 0.85:
                    reward += 1
                elif 0.25 < pos[0] < 0.35 or 0.5 < pos[0] < 0.6:
                    reward -= 1

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


