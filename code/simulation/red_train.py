import os
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor


from stable_baselines3.common.callbacks import BaseCallback
from env_only_red import GarbageSortingEnvOnlyRed

class PrintRewardCallback(BaseCallback):
    def __init__(self, env, reward_threshold):
        super(PrintRewardCallback, self).__init__()
        self.env = env
        self.reward_threshold = reward_threshold

    def _on_step(self) -> bool:
        if self.n_calls % self.model.n_steps == 0:
            episode_rewards = self.env.get_attr('get_episode_rewards')[0]()
            if len(episode_rewards) > 0:
                last_episode_reward = episode_rewards[-1]
                print(f"Reward of the last episode before update {self.num_timesteps}: {last_episode_reward:.2f}")

                # if last_episode_reward >= self.reward_threshold:
                #     print("Reward threshold reached, stopping training.")
                #     # Save the model when the reward threshold is reached
                #     self.model.save(f"high_reward_model_{self.num_timesteps}")
                #     return False
        return True


def main():
    # Create the environment
    env = GarbageSortingEnvOnlyRed(gui=False)
    env = Monitor(env)  # Wrap the environment with Monitor
    env = DummyVecEnv([lambda: env])

    training_timesteps = 500000
    print('Training timesteps is: ',training_timesteps)
    # 1.4 near distance, not fall punish
    # model_name = 'pusher_like_1m_random'
    # 1.4 near distance,  fall punish -100, and huge reward
    model_name = 'test_near_0.5m'

    # Instantiate the PPO agent
    model = PPO("MlpPolicy", env, 
                verbose=1, 
                learning_rate=1e-4, 
                n_steps=900, batch_size=900, 
                gamma=0.99, gae_lambda=0.95,
                ent_coef=0.1,
                tensorboard_log="./ppo_garbage_sorting_tensorboard/")

    # Create the callback
    reward_threshold = 10000  # Set the reward threshold for early stopping
    callback = PrintRewardCallback(env, reward_threshold)

    # Train the agent
    model.learn(total_timesteps=int(training_timesteps), callback=callback)

    # Save the trained model
    model.save(model_name)

if __name__ == "__main__":
    main()

