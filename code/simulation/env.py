import os
import time
import pybullet as p
import gym
from gym import spaces
import numpy as np
import random
import pybullet_data
from robot import UR5, CONVEYOR, GARBAGE
from utils import saveImg
from PIL import Image
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

        # Define action and observation spaces
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.num_robots * 6,), dtype=np.float32)
        # robot, 
        observation_space_shape = 6*self.num_robots + 3 + 4 * self.num_garbage
        self.observation_space = spaces.Box(low=0, high=1, shape=(observation_space_shape,), dtype=np.float32)


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
        p.loadURDF("plane.urdf")

        x_color = [1, 0, 0]  
        y_color = [0, 1, 0]  
        z_color = [0, 0, 1] 

        p.addUserDebugLine([0, 0, 2], [5, 0, 2], x_color, 5, 20000)
        p.addUserDebugLine([0, 0, 2], [0, 5, 2], y_color, 5, 20000)
        p.addUserDebugLine([0, 0, 0], [0, 0, 10], z_color, 5, 20000)


        # Not necessary to initialize any garbage here, the reset includes garbage


        # ------------------------- initialize robotic arm ------------------------------
        self.robot_arm_ids = p.loadURDF('../urdf/ur5_robotiq_85.urdf', [0 - self.move_right,-0.8,0.47], p.getQuaternionFromEuler([0, 0, 0]))
        # ------------------------- robotic arm ------------------------------
   

        # ------------------------- initialize conveyor belt ------------------------------
        conveyor_pos = [0.56, 0, 0.1]
        conveyor_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.conveyor_id = p.loadURDF("../simulation/urdf/block.urdf", conveyor_pos, conveyor_ori)
        conveyor_speed = self.conveyor_speed   
        conveyor_joint_index = 0 
        p.setJointMotorControl2(self.conveyor_id, conveyor_joint_index, p.VELOCITY_CONTROL, targetVelocity=conveyor_speed)
        # ------------------------- conveyor belt ------------------------------


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

        print('--'*10)
        print("Joint positions:\n", joint_positions)
        print("Effector position:\n", effector_position)
        print("Garbage positions:\n", garbage_positions)
        print("Garbage type:\n", garbage_type)
        print('--'*10)
        

        # Calculate reward
        reward = self.calculate_reward(observation)

        # Check if the episode is done
        done = self.is_done(observation)

        return observation, reward, done, {}

    def reset(self):

        # Reset robot arm joint positions
        joint_positions = [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # UR5 example joint positions
        for i, position in enumerate(joint_positions):
            p.resetJointState(self.robot_arm_ids, i, position)

        # Reset and re-generate garbage objects
        self.garbage = GARBAGE(self.num_garbage)
        self.garbage.generateGarbage()

        box_dicts = [d for d in self.garbage.garbageData if d['boxId'] is not None]
        type_ = box_dicts[0]['type']
        name_ = box_dicts[0]['name']
        # print('-'*30)
        print('Name is:', name_)
        print('Type is: ',type_)
        print('-'*30)
        
        # Get observation
        observation = self.get_observation()

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
            print('garbage position and oritation is:')
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
        return False

    def calculate_reward(self, observation):
        return 1

    
def main():

    debug = False
    gui = True

    p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    env = GarbageSortingEnv()
    total_episodes = 10
    max_steps_per_episode = 1500

    for episode in range(total_episodes):
        print(f"Episode: {episode + 1}")
        obs = env.reset()
        done = False
        episode_reward = 0

        for step in range(max_steps_per_episode):
            action = env.action_space.sample()  # Sample random action for the robotic arm
            obs, reward, done, _ = env.step(action)

            if debug: 
                print("Observation:", obs)
                print("Reward:", reward)
                print("Done:", done)

            episode_reward += reward

            if debug: print("episode_reward:", episode_reward)

            if done:
                break

            time.sleep(1./100.)

        # print(f"Episode reward: {episode_reward}")
        
        saveImg()

    env.close()





if __name__ == "__main__":
    main()
