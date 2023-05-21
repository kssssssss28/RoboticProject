
import time
from two_arms_env_blue import GarbageSortingEnvOnlyRedV2
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

# delay in for loop
time_delay = 300


# load the trajectory trained by RL
with open('RL_model_trajectory2.pkl', 'rb') as f:
    traj = pickle.load(f)

# find the start action of robotic arm according to the RL model pre-trained
def find_nearest_array_index(arrays, x):
    array_distances = [np.abs(array[0] - x) for array in arrays]
    nearest_index = np.argmin(array_distances)
    return nearest_index, arrays[nearest_index][1]

# calculate the time steps the garbage requires to reach the pushing point
def cal_garbage_move_to_p_step(arm_y, initial_y):
    distance = abs(initial_y - arm_y)
    return int(distance/0.0039322)  #0.0037525722

# compute how many steps the robotic arm need to pause to approach the best pushing timing\
def compute_pause_steps(input_x, traj):
    nearest_index, y = find_nearest_array_index(traj, input_x + 0.0825146)
    garbage_steps = cal_garbage_move_to_p_step(y, -4)

    return garbage_steps - nearest_index

# According to the trained RL model, generate next action
def predict_next_action(array, steps, stop = 1):
    pattern = [array.copy()]
    
    for _ in range(steps - 1):
        if  _ < stop:
            mul = 0
        else:
            mul = 1

        new_array = np.zeros(6, dtype=np.float32)
        new_array[0] = array[0] + 0.0009*mul
        new_array[1] = array[1] - 0.0035*mul
        new_array[2] = array[2] - 0.0001*mul
        new_array[3] = array[3] + 0.0011*mul
        new_array[4] = array[4] + 0.0010*mul
        new_array[5] = array[5] - 0.0005*mul
        
        pattern.append(new_array)
        array = new_array
    
    return pattern

start_array2 = np.array([-0.6935777  , 1.0789974 , -0.03136498 ,-0.02120538, -0.37908894, 0.88832015], dtype=np.float32)

refresh = 1000



def main():
    number_of_garbage = 15

    # initial random location for garbages to test how robotic arm can adapt to the environment changes
    random_numbers = [random.uniform(0.55, 0.85) for _ in range(number_of_garbage)]
    env = GarbageSortingEnvOnlyRedV2(gar_x = random_numbers,debug=False)
    total_episodes = 1
    max_steps_per_episode = 16000
    process_num = 600

    pause_list = []
    move_list = [] 

    for i in range(number_of_garbage):
        pause_steps_num = compute_pause_steps(random_numbers[i], traj)
        pause_list.append(pause_steps_num)
        move_list.append(predict_next_action(start_array2, pause_steps_num + process_num,stop = pause_steps_num))

 
    stay = predict_next_action(start_array2, 1000,stop = 1000)
  

    for episode in range(total_episodes):
        print(f"Episode: {episode + 1}")
        # obs = env.reset()
        done = False
  

        cnt = -1
        for step in range(max_steps_per_episode):
          

            if step % refresh == 0:
                
                cnt+=1

            step_ = step - cnt* refresh
            ps = pause_list[cnt]

            if step_ < ps+process_num - 1:
                action1 = move_list[cnt][step_]

            else:
                action1 = stay[step_]

            obs, reward, done, _ = env.step(action1)
        
          
            if done:
                break
            time.sleep(1./time_delay)


        
        saveImg()

    env.close()





if __name__ == "__main__":
    main()
