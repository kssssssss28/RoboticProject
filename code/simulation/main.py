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

    box = GARBAGE(10)
    box.generateGarbage()
    con = CONVEYOR()
    robot = UR5()

    starSimulation()

if __name__ == '__main__':
    main()
