import os
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from env import GarbageSortingEnv

def main():
    # Create the environment
    env = GarbageSortingEnv()
    env = DummyVecEnv([lambda: env])
<<<<<<< HEAD
    model_name = 'ppo_0.15m'

=======
    model_name = 'ppo_conveyor_moving_KS'
    
>>>>>>> cab21b979a822e831406bdf9110b82b6fc560d36
    # Instantiate the PPO agent
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_garbage_sorting_tensorboard/")

    # Train the agent
<<<<<<< HEAD
    training_timesteps = 150000
=======
    training_timesteps = 300000
>>>>>>> cab21b979a822e831406bdf9110b82b6fc560d36
    model.learn(total_timesteps=int(training_timesteps))

    # Save the trained model
    model.save(model_name)

    # Evaluate the trained model
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)

    print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")

if __name__ == "__main__":
    main()
