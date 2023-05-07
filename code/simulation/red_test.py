import os
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
# from env_only_red import GarbageSortingEnvOnlyRed
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
    n_eval_episodes=30

    for episode in range(n_eval_episodes):

        obs = env.reset()
        done = False
        total_reward = 0
        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            # print('reward:', reward)
            total_reward += reward
            # print("episode_reward:", total_reward)
            print("action:", action)
            # print("step:", episode)



            time.sleep(1/140)

        # print(f"Episode {episode + 1}: Total Reward = {total_reward}")



    # # Evaluate the trained model
    # mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)

    # print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")

if __name__ == "__main__":
    main()

