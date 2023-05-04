import os
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from envNotSparse import GarbageSortingEnv
#from envSparse import GarbageSortingEnv
#from envOnlyDes import GarbageSortingEnv
def main():
    # Create the environment
    env = GarbageSortingEnv(gui=True)
    env = DummyVecEnv([lambda: env])
    model_name = '../model/ppo_armPickUpBox'
    model = PPO.load(model_name)

    # Evaluate the trained model
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)

    print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")

if __name__ == "__main__":
    main()
