import os
import numpy as np
import pybullet as p
from tqdm import tqdm
import time
import math
import pybullet as p
from robot import UR5, CONVEYOR, GARBAGE
from utils import initWorld,starSimulation, generateGarbage


def main():
    GUI=True
    initWorld(GUI)
    robot = UR5()
    con = CONVEYOR()
    box = generateGarbage()
    starSimulation()

if __name__ == '__main__':
    main()
