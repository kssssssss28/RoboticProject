import os
import numpy as np
import pybullet as p
from tqdm import tqdm
import time
import math
import pybullet as p
from robot import UR5, CONVEYOR, GARBAGE
from utils import initWorld,starSimulation


def main():
    GUI=True
    initWorld(GUI)

    number = 3
    robots = UR5(number)

    for index in range(number):
        #### 机器人的id是从1 开始的
        robots.moveArm(robots.robots[index]["id"],[1,index - 1,1],[0,0,1])

    con = CONVEYOR()
    starSimulation()

if __name__ == '__main__':
    main()
