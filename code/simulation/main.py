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
    # 是否使用GUI  摄像头的位置 摄像头的高度
    initWorld(GUI,[1, .5, 0], 4)

    number = 4
    # robots = UR5(number)

    # for index in range(number):
    #      #### 机器人的id是从1 开始的
    #      robots.moveArm(robots.robots[index]["id"],[1,index - 1,1],[0,0,1])

    con = CONVEYOR()
    #         垃圾的数量 是不是要拍照 产生垃圾的间隔&拍照间隔
    starSimulation(5,True, 4000)



if __name__ == '__main__':
    main()
