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
desRed = [1.2,.7,0.8]
desBlue = [-1.2,.7,0.8]


class GARBAGE():

    def __init__(self, number):
        garbageMap = ["red","blue","green"]
        self.threePath = [0.3, 0.55, 0.8, 0.3, 0.8]
        self.threePathy = [0, -.5, -1, -1.5, -2]
        with open('../simulation/data/data.json', 'r') as fcc_file:
             garbageInfo = list(json.load(fcc_file))

        garbageInfo.pop(0)
        self.garbageData = []
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.number = number
        count = 0
        self.onConveyor = []
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
        x = random.choice(self.threePath)
        y = random.choice(self.threePathy)
        self.threePath.remove(x)
        self.threePathy.remove(y)
        rdPosition = [x ,y-.1, .4]
        garbage["startPos"] = rdPosition
        startPos = garbage["startPos"]
        startOri = garbage["startOri"]
        boxId = p.loadURDF(path, startPos, startOri)
        p.changeDynamics(boxId,-1,mass = 5)
        item = dict()
        item["boxId"] = boxId
        item["type"] = garbage["type"]
        item["pos"] = startPos
        self.onConveyor.append(item)
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
        self.grab= 0
        self.holding = 0

        self.garbage = GARBAGE(self.num_garbage)
        # self.reset()
        
        # Define action and observation spaces
        self.action_space = spaces.Box(low=-2, high=2, shape=(6,), dtype=np.float32)
        # desRed = [1.2,.7,0.8]
        # desBlue = [-1.2,.7,0.8]
        self.action_space.low[0] = 0
        self.action_space.low[1] = 0
        self.action_space.low[2] = 0
        self.action_space.high[0] = 1
        self.action_space.high[1] = 1
        self.action_space.high[2] = 1
        self.action_space.low[3] = -1.5
        self.action_space.low[4] = -1.5
        self.action_space.low[5] =  0.9
        self.action_space.high[3] = 1.5
        self.action_space.high[4] = 1.5
        self.action_space.high[5] = 1.5
        
        
        
        
        # robot,           gripper position garbage pos gripper status distance(g and d) distance(g and g) 
        observation_space_shape =  3 + 3 + 1 + 1 + 1
        
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

        # Load the environment modelsdebug
        self.load_models()

    def load_models(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        ground_id = p.loadURDF(pybullet_data.getDataPath() + "/plane.urdf")
        line = p.addUserDebugLine([0,0,0], desBlue, [0, 0, 1], 5, 300)
        line = p.addUserDebugLine([0,0,0], desRed, [1, 0, 0], 5, 300)
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
        target_Position = [action[3], action[4], action[5]]
        effector_position = [observation[3], observation[4], observation[5]]
        garbage_positions = [observation[0], observation[1], observation[2]]
        
        ggDistance = observation[6]
        gdDistance = observation[7]
        targetPosition = [action[0], action[1], action[2]]
        type_ = observation[8]
        line = p.addUserDebugLine(targetPosition, target_Position, [0, 1, 0], 5, 2)

        if ggDistance <= 0.2:
            self.kuka.grab(0,ggDistance,self.garbage.onConveyor[0]["boxId"])
            area = type_
            #self.kuka.move2Area(0,area)
            self.holding = True
            self.grab = self.grab + 1 
            self.kuka.moveArm(0, 9999, target_Position)  
            if gdDistance <= 0.4:
                self.kuka.release()

                p.removeBody(self.garbage.onConveyor[0]["boxId"])
                self.movingDone = True
                self.garbage.onConveyor.pop(0)
                
  
        else: 
            # Apply action to the robotic arm
            self.kuka.moveArm(0,9999,targetPosition)
            

           
        
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
        self.grab = 0
        # Reset robot arm joint positions
        joint_positions = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # UR5 example joint positions
        for i, position in enumerate(joint_positions):
            p.resetJointState(self.robot_arm_ids, i, position)

        # Remove previous garbage objects
        for garbage in self.garbage.garbageData:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None
                
        for garbage in self.garbage.onConveyor:
            if garbage["boxId"] is not None:
                p.removeBody(garbage["boxId"])
                garbage["boxId"] = None

        # Reset and re-generate garbage objects
        self.garbage = GARBAGE(self.num_garbage)
        
        for i in range(5):
            self.garbage.generateGarbage()

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
        debug = False  

        # Get end-effector position
        effector_position = p.getBasePositionAndOrientation(2)[0]
        
        
        
        for i in range(len(self.garbage.onConveyor)):
            gtype = self.garbage.onConveyor[i]["type"]
            self.garbage.onConveyor[i]["distance"] = euclidean_distance(effector_position, 
                                                                        p.getBasePositionAndOrientation(self.garbage.onConveyor[i]["boxId"])[0]
                                                                        )
            
        self.garbage.onConveyor = sorted(self.garbage.onConveyor, key=lambda elem: elem['distance'], reverse=False)
        
        garbage = self.garbage.onConveyor

        garbage_positions = p.getBasePositionAndOrientation(garbage[0]["boxId"])[0]

        # Get distance between garbage and gripper
        garbageDistanceGripper = euclidean_distance(effector_position, garbage_positions)


        # color = 
        boxType = garbage[0]["type"]
        type_ = boxType
        
        
        # Get distance between garbage and destination
        
        if boxType == 0:
            des = desRed
        else:
            des = desBlue
            
        garbageDistanceDes = euclidean_distance(des, garbage_positions)
       
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
        type_ = np.array([type_])

        # Concatenate observations  3                    3                   1                  1                       1
        observation = np.concatenate((garbage_positions, effector_position, garbageDistanceGripper, garbageDistanceDes, type_))
        
        return observation


    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

    def is_done(self, observation):
        
        
        garbage_positions =[observation[0], observation[1], observation[2]]
        gdDistance = observation[7]
        type_ = observation[8]
        
        # Check if garbage has fallen off the conveyor
        for g in self.garbage.onConveyor:
            if g["pos"][0] >= 1.5 or g["pos"][0] <= 0.2:
                return True
            if g["pos"][1] >= 1.1:
                return True
    

        if garbage_positions[1] >= 1.5:
            return True
        
        if garbage_positions[0] >= 1.5:
            return True
        
        if garbage_positions[0] <= 0.2:
            return True
        
        if len(self.garbage.onConveyor) == 0:
            return True
        
        # Check if the robotic arm has successfully moved the garbage to its correct line
        success = False
        
        if gdDistance <= 0.4:
            print("===success===")
            success = True
        
    
        if self.steps >= 500:
            return True

        return success

    def calculate_reward(self, observation):
        
        gdDistance = observation[7]
        type_ = observation[8]
        garbagePosition = [observation[0], observation[1], observation[2]]
        

        reward = 0
        
        
        grab = 0
        
        wrongDis = 0
        
        dis = 0
        print("gdDis", gdDistance)
        if self.holding:
            grab = 1
            if type_ == 1:
                wrongDis = euclidean_distance(desRed, garbagePosition)
            else:
                wrongDis = euclidean_distance(desBlue, garbagePosition)

            
            if gdDistance >= 2:
                dis = -2
                
            
            elif gdDistance < 2 and gdDistance >= 1.9:
                dis = -1.9

            elif gdDistance < 1.9 and gdDistance >= 1.85:
                dis = -1.8
            
            elif gdDistance < 1.85 and gdDistance >= 1.8:
                dis = -1.7
                
            elif gdDistance < 1.8 and gdDistance >= 1.75:
                dis = -1.5
                
            elif gdDistance < 1.75 and gdDistance >= 1.7:
                dis = -1.3
                
            elif gdDistance < 1.6 and gdDistance >= 1.4:
                dis = -1.1
            
            elif gdDistance < 1.4 and gdDistance >= 1.3:
                dis = -1
                
            elif gdDistance < 1.3 and gdDistance >= 1.2:
                dis = -.8
                
            elif gdDistance < 1.2 and gdDistance >= 1.1:
                dis = -.7
                
            elif gdDistance < 1.1 and gdDistance >= 1:
                dis = -0.6    
                               
            elif gdDistance < 1 and gdDistance >= 0.95:
                dis = -0.4
            
            elif gdDistance < .95 and gdDistance >= 0.9:
                dis = 0

            elif gdDistance < 9 and gdDistance >= 0.85:
                dis = 0.3
            
            elif gdDistance < .85 and gdDistance >= 0.8:
                dis = 0.5
                
            elif gdDistance < 0.8 and gdDistance >= 0.75:
                dis = 0.7
                
            elif gdDistance < 0.75 and gdDistance >= 0.7:
                dis = 0.9
                
            elif gdDistance < 0.6 and gdDistance >= .4:
                dis = 1.3
            
            elif gdDistance < 0.4 and gdDistance >= .1:
                dis = 1.8
            
            
        reward =  grab + dis
        
        print("=====reward=====", wrongDis, grab, dis, reward)
            
        
        
        
        

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


