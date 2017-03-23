import cv2
import numpy as np
import time
import robotutil
import Workspace1
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser

robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.home()

robot.flyManipAir(True)
