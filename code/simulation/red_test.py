import os
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
# from env_only_red import GarbageSortingEnvOnlyRed
import pickle
from env_only_red_v2 import GarbageSortingEnvOnlyRedV2
'''
working model

'''


# V2
def main():
    # Create the environment
    env = GarbageSortingEnvOnlyRedV2(gui= True)
    env = DummyVecEnv([lambda: env])
    # model_name = 'pusher_like_0.5m'
    # model_name = 'pusher_like_1m_random'
    model_name = 'can_follow_garbage_moving_0.5m'



    model = PPO.load(model_name)
    n_eval_episodes=1

    for episode in range(n_eval_episodes):

        obs = env.reset()
        done = False
        total_reward = 0
        step = 0
        action_list = []
        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            # print('reward:', reward)
            total_reward += reward
            # print("episode_reward:", total_reward)
            step += 1
            print('In step ' , step)

            if step > 435 and step < 860:
                action_list.append(action[0])
            print("action:", action)
            # print("step:", episode)



            time.sleep(1/100)
        
        with open('push.pkl', 'wb') as f:
            pickle.dump(action_list, f)


        # print(f"Episode {episode + 1}: Total Reward = {total_reward}")



   

if __name__ == "__main__":
    main()

