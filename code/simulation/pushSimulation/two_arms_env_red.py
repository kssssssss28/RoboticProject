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

    def __init__(self, number, gar_x):
        self.threePath = [0.3, 0.55]
        with open('../simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))

        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number
        self.gar_x = gar_x
        self.cur = 0

        for i in garbageInfo:  
            type = 'red'
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
    
        rdPosition = [self.gar_x[self.cur],-4,.4] 
        print('Current random location of garbage is: \n', self.gar_x[self.cur])
        garbage["startPos"] = rdPosition
        startPos = garbage["startPos"]
        startOri = garbage["startOri"]
        boxId = p.loadURDF(path, startPos, startOri)
        p.changeDynamics(boxId,-1,mass = 5)
        self.garbageData[rd]["boxId"] = boxId
        self.cur += 1

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
    def __init__(self, gar_x = 0.55, gui=True, debug = False, num_robots=1, num_garbage=1, camera_pos=[0.75, -2.7, -1], camera_distance=4, conveyor_speed=1, garbage_delay=4000):
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
        self.gar_x = gar_x
        self.garbage = GARBAGE(self.num_garbage, self.gar_x)
        self.refresh = 1000
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_robots * 6,), dtype=np.float32)
        observation_space_shape = 12*self.num_robots + 6 + 4 * self.num_garbage
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
        self.reset()

    def get_effector_position1(self, action):
        for i, target_position in enumerate(action):
            p.setJointMotorControl2(self.robot_arm_ids, i, p.POSITION_CONTROL, targetPosition=target_position)
        
        observation = self.get_observation()
        effector_position1 = observation[6:9]

        return effector_position1


    def load_models(self):
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        

        x_color = [1, 0, 0]  
        y_color = [0, 1, 0]  
        z_color = [0, 0, 1] 

        p.addUserDebugLine([0, 0, 2], [5, 0, 2], x_color, 5, 20000)
        p.addUserDebugLine([0, 0, 2], [0, 5, 2], y_color, 5, 20000)
        p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)


        # ------------------------- initialize robotic arm ------------------------------
        self.robot_arm_ids = p.loadURDF('../simulation/urdf/ur5_robotiq_85.urdf', [0 - self.move_right,-2.3,0.5], p.getQuaternionFromEuler([0, 0, -.6]))        # -1.6 
        self.robot_arm_ids2 = p.loadURDF('../simulation/urdf/ur5_robotiq_85.urdf', [1.2, 1,0.47], p.getQuaternionFromEuler([0, 0, 1]))        
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

    def step(self, action1):
 
        for i, target_position in enumerate(action1):
            p.setJointMotorControl2(self.robot_arm_ids, i, p.POSITION_CONTROL, targetPosition=target_position)
        
  
        # Step simulation
        p.stepSimulation()

        # Move the conveyor beltgar_x
        conveyor_id = self.conveyor_id
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.setJointMotorControl2(conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)

        # Get observation
        observation = self.get_observation()

    

        self.steps += 1

        if self.steps % self.refresh == 0:
            for garbage in self.garbage.garbageData:
                if garbage["boxId"] is not None:
                    p.removeBody(garbage["boxId"])
                    garbage["boxId"] = None

            self.garbage.generateGarbage()

        # Calculate reward
        reward = self.calculate_reward(observation)

        self.accumulated_reward += reward
        done = self.is_done(observation)

            

        return observation, reward, done, {}



    def reset(self):
        

        self.steps = 0
        self.accumulated_reward = 0

        joint_positions = [ 0.   ,      -1.64401707 ,-0.07291283 , 0.42961074 , 0.19542162 ,-0.7959876 ]
        
        for i, position in enumerate(joint_positions):
            p.resetJointState(self.robot_arm_ids, i, position)
            p.resetJointState(self.robot_arm_ids2, i, position)
            

 
        for garbage in self.garbage.garbageData:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None


        
        self.garbage.generateGarbage()


        # Reset conveyor belt
        conveyor_joint_index = 0
        conveyor_speed = self.conveyor_speed
        p.resetJointState(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL,targetVelocity=conveyor_speed)
        p.createConstraint(self.ground_id, -1, self.conveyor_id, -1, p.JOINT_FIXED, [0, 0, 0], [0.56, 0, 0.1], [0, 0, 0]) # fix the conveyor


        # Get observation
        observation = self.get_observation()

        return observation

    
    def get_observation(self):
        debug = False    

        # Get joint states
        joint_states1 = p.getJointStates(self.robot_arm_ids, range(6))  
        joint_positions1 = np.array([state[0] for state in joint_states1])
        effector_position1, _1 = p.getLinkState(self.robot_arm_ids, 5)[:2] 

        joint_states2 = p.getJointStates(self.robot_arm_ids2, range(6))  
        joint_positions2 = np.array([state[0] for state in joint_states2])
        effector_position2, _2 = p.getLinkState(self.robot_arm_ids2, 5)[:2] 


        # Get garbage objects' positions and orientations
        garbage_positions_orientations = [p.getBasePositionAndOrientation(garbage["boxId"]) for garbage in self.garbage.garbageData if garbage["boxId"] is not None]
        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']

        garbage_positions = np.array([pos for pos, _ in garbage_positions_orientations])

        observation = np.concatenate((joint_positions1, effector_position1, joint_positions2, effector_position2, garbage_positions.flatten(),[type_]))
        

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
    
        success = False
         
        if self.steps >= self.total_steps:
            return True
        
        
        return success
    
    def is_fall(self, current_z):
        if current_z < 0.37 or current_z > 1:
            return True
        return False

    # in simulation, no need to calculate reward to save computation resource
    def calculate_reward(self, observation):

        return 0
    

