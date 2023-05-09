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
        # garbageMap = ["red","blue","green"]
        self.threePath = [0.3, 0.55]
        with open('../simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))

        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number

        for i in garbageInfo:  
            # type = garbageMap[int(i["obj_id"])]
            type = 'red'
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
        rdPath = random.randint(0,1)
        random_number = random.uniform(0.3, 0.85)
        rdPosition = [random_number,-3,.4]
        garbage["startPos"] = rdPosition
        startPos = garbage["startPos"]
        startOri = garbage["startOri"]
        boxId = p.loadURDF(path, startPos, startOri)
        p.changeDynamics(boxId,-1,mass = 5)
        self.garbageData[rd]["boxId"] = boxId

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

class GarbageSortingEnvOnlyRedV2(gym.Env):
    def __init__(self, gui=True, debug = False, num_robots=1, num_garbage=1, camera_pos=[1, .5, 0], camera_distance=4, conveyor_speed=1, garbage_delay=4000):
        super().__init__()

        # Initialize environment parameters
        self.gui = gui
        self.num_robots = num_robots
        self.num_garbage = num_garbage
        self.camera_pos = camera_pos
        self.camera_distance = camera_distance
        self.conveyor_speed = conveyor_speed
        self.garbage_delay = garbage_delay
        self.move_right = 0.12 
        self.steps = 0
        self.total_steps = 5500
        self.previous_y = 0
        self.accumulated_reward = 0
        self.debug = debug
        self.garbage = GARBAGE(self.num_garbage)
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_robots * 6,), dtype=np.float32)
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
            cameraPitch=239,
            cameraTargetPosition=self.camera_pos
        )
        self.ground_id = p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf")
        # Load the environment models
        self.load_models()

    def load_models(self):
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        

        x_color = [1, 0, 0]  
        y_color = [0, 1, 0]  
        z_color = [0, 0, 1] 

        p.addUserDebugLine([0, 0, 2], [5, 0, 2], x_color, 5, 20000)
        p.addUserDebugLine([0, 0, 2], [0, 5, 2], y_color, 5, 20000)
        p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)


        # Not necessary to initialize any garbage here, the reset includes garbage


        # ------------------------- initialize robotic arm ------------------------------
        self.robot_arm_ids = p.loadURDF('../simulation/urdf/ur5_robotiq_85.urdf', [0 - self.move_right,-2.3,0.47], p.getQuaternionFromEuler([0, 0, 0]))        
        self.robot_arm_ids2 = p.loadURDF('../simulation/urdf/ur5_robotiq_85.urdf', [1.2, 1,0.47], p.getQuaternionFromEuler([0, 0, 3.15]))        
        # ------------------------- robotic arm ------------------------------
   

        # ------------------------- initialize conveyor belt ------------------------------
        conveyor_pos = [0.56, 0, 0.1]
        conveyor_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.conveyor_id = p.loadURDF("../simulation/urdf/block.urdf", conveyor_pos, conveyor_ori)
        conveyor_speed = self.conveyor_speed   
        conveyor_joint_index = 0 
        p.setJointMotorControl2(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)
        # ------------------------- fix the conveyor belt ------------------------------
        p.createConstraint(self.ground_id, -1, self.conveyor_id, -1, p.JOINT_FIXED, [0, 0, 0], [0.56, 0, 0.1], [0, 0, 0])

    def step(self, action1, action2):
        # debug = True
        # Apply action to the robotic arm
        for i, target_position in enumerate(action1):
            p.setJointMotorControl2(self.robot_arm_ids, i, p.POSITION_CONTROL, targetPosition=target_position)
        
        for i, target_position in enumerate(action2):
            p.setJointMotorControl2(self.robot_arm_ids2, i, p.POSITION_CONTROL, targetPosition=target_position)
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

        self.steps += 1

        # Calculate reward
        reward = self.calculate_reward(observation)

        self.accumulated_reward += reward
        done = self.is_done(observation)
        '''
        if do nothing, the reward would be -3000
        fall would lead to bad result, but can encourage the agent to move the object to correct place.
        so, accumulated fall should be less than -3000?
        '''
        # if self.accumulated_reward < -1999:
        #     done = True
        # else:
        #     # Check if the episode is done
        #     done = self.is_done(observation) or self.steps >= self.total_steps

        if self.debug: print('Current reward: ', reward)
        if self.debug: print('Accumulated reward: ',self.accumulated_reward)
            
        # print('reward:', reward)
        return observation, reward, done, {}



    def reset(self):
        

        self.steps = 0
        self.accumulated_reward = 0
        # Reset robot arm joint positions
        joint_positions = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # UR5 example joint positions
        for i, position in enumerate(joint_positions):
            p.resetJointState(self.robot_arm_ids, i, position)
            p.resetJointState(self.robot_arm_ids2, i, position)
            

        # Remove previous garbage objects
        for garbage in self.garbage.garbageData:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None

        # Reset and re-generate garbage objects
        self.garbage = GARBAGE(3)
        for i in range(3):
            self.garbage.generateGarbage()

        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']

        # Reset conveyor belt
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.resetJointState(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL,targetVelocity=conveyor_speed)
        p.createConstraint(self.ground_id, -1, self.conveyor_id, -1, p.JOINT_FIXED, [0, 0, 0], [0.56, 0, 0.1], [0, 0, 0]) # fix the conveyor


        # Get observation
        observation = self.get_observation()

        joint_positions = observation[:self.num_robots*6].reshape((self.num_robots, 6))
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]

        if self.debug: 
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
        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']

        garbage_positions = np.array([pos for pos, _ in garbage_positions_orientations])

        observation = np.concatenate((joint_positions, effector_position, garbage_positions.flatten(),[type_]))
        

        return observation


    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()
    
    def is_garbage_reachable(self, effector_position, garbage_position):
        distance = self.calculate_distance(effector_position, garbage_position)
        
        if 0.122 <= distance <= 0.9062:
            return True
        else:
            return False

    # calculate the distance between the end effector and the garbage object
    def calculate_distance(self, effector_position, garbage_position):
        return np.linalg.norm(np.array(effector_position) - np.array(garbage_position))

    def is_back(self, current_y):
        if current_y < self.previous_y:
            return True
        
        return False
    


    def is_done(self, observation):
        
        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]
        current_y = 0

        success = False
         
        if self.steps >= self.total_steps:
            return True
        
        
        return success
    
    def is_fall(self, current_z):
        if current_z < 0.37 or current_z > 1:
            return True
        return False

    def calculate_reward(self, observation):

        garbage_positions = observation[self.num_robots*6+3:self.num_robots*6+3+self.num_garbage*4].reshape((self.num_garbage, 4))
        garbage_type = observation[-1]
        effector_position = observation[self.num_robots*6:self.num_robots*6+3]

        reward = 0
        current_y = 0
        debug = False
        left_boundary = 0.9
        for i, pos in enumerate(garbage_positions):

            # encourage the effector to touch with the object in order to move it.
            distance = self.calculate_distance(effector_position, pos[:3])

            # whether the garbage is within the operational range of the robotic arm. 
            reachable = self.is_garbage_reachable(effector_position, garbage_positions[0][:3])
            current_y = pos[1]
            current_z = pos[2]

            dis_reward = 0
            back = self.is_back(current_y)
            fall = self.is_fall(current_z)

            x_ = pos[0]
            if debug: print('current_x is: ', x_)

            if fall:

                dis_reward = -10
                if debug: print('Fall, current z is: ',current_z)

            else:
                if 0.56 <= x_ < left_boundary:
                    dis_reward = (1/(left_boundary - x_))

                    if debug: print('Encourage')

                elif 0 < x_< 0.54:
                    # dis_reward = (0.55 - x_) * (-10)
                    dis_reward = -1
                    if debug: print('punish')
                elif 0.54 < x_< 0.56:
                    dis_reward = 0
                    if debug: print('Nothing')
                else:
                    dis_reward = -1
                    if debug: print('Out')
                
            reward += dis_reward

            if self.steps > 600 and 0.54 < x_< 0.56:
                reward -= 30
            
            if 0.85 < x_ < left_boundary and self.steps > 600 and not fall:
                reward += 100

        self.previous_y = current_y

        return reward
    

