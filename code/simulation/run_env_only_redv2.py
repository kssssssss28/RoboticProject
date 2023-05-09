
import time
from env_only_red_v2 import GarbageSortingEnvOnlyRedV2
import os
import time
import pybullet as p
import gym
from gym import spaces
import numpy as np
import random
import pybullet_data
import json
from utils import saveImg
import pickle

def generate_move_left_pattern(array, steps):
    pattern = [array.copy()]
    
    for _ in range(steps - 1):
        new_array = np.zeros(6, dtype=np.float32)
        new_array[0] = array[0] - 0.0009
        new_array[1] = array[1] + 0.0035
        new_array[2] = array[2] + 0.0001
        new_array[3] = array[3] - 0.0011
        new_array[4] = array[4] - 0.0010
        new_array[5] = array[5] + 0.0005
        
        pattern.append(new_array)
        array = new_array
    
    return pattern


def generate_move_right_pattern(array, steps):
    pattern = [array.copy()]
    
    for _ in range(steps - 1):
        new_array = np.zeros(6, dtype=np.float32)
        new_array[0] = array[0] - 0.0009
        new_array[1] = array[1] - 0.0035
        new_array[2] = array[2] + 0.0001
        new_array[3] = array[3] - 0.0011
        new_array[4] = array[4] - 0.0010
        new_array[5] = array[5] + 0.0005
        
        pattern.append(new_array)
        array = new_array
    
    return pattern

start_array = np.array([-0.15898392, -1.0, -0.09076598, 0.63219655, 0.37908894, -0.88832015], dtype=np.float32)
num_arrays = 5000
def main():

    debug = False

    env = GarbageSortingEnvOnlyRedV2(debug=False)
    total_episodes = 10
    max_steps_per_episode = 1500

    # with open('push.pkl', 'rb') as f:
    #     loaded_list = pickle.load(f)
    # loaded_list = loaded_list[::-1]
    move_left = generate_move_left_pattern(start_array, num_arrays)
    print(move_left[-1])
    # move_right = generate_move_right_pattern(np.array(move_left[-1], dtype=np.float32), num_arrays)
    move_right = generate_move_right_pattern(start_array, num_arrays)
    
    for episode in range(total_episodes):
        print(f"Episode: {episode + 1}")
        obs = env.reset()
        done = False
        episode_reward = 0

        for step in range(max_steps_per_episode):
            
            # action = env.action_space.sample()  \
            # if step < 300:
            #     action = move_left[step]
            # else:
            #     step = step-300
            #     action = move_right[step]
            action = move_right[step]
            print('step is: ', step)
            print('action is: ', action)
            obs, reward, done, _ = env.step(action)

            if debug: 
                print("Observation:", obs)
                print("Reward:", reward)
                print("Done:", done)

            episode_reward += reward

            # print("episode_reward:", episode_reward)
            # print(step)
            if done:
                break
            # print('step is:', step)
            time.sleep(1./100.)


        
        saveImg()

    env.close()





if __name__ == "__main__":
    main()
