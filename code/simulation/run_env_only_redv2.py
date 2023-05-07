
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

def main():

    debug = False

    env = GarbageSortingEnvOnlyRedV2(debug=True)
    total_episodes = 10
    max_steps_per_episode = 1500

    for episode in range(total_episodes):
        print(f"Episode: {episode + 1}")
        obs = env.reset()
        done = False
        episode_reward = 0

        for step in range(max_steps_per_episode):
            action = env.action_space.sample()  
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
