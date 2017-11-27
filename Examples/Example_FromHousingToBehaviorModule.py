## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: Example_HousingToBehaviorModule.py
#  Description: Example routine calling highest-level functions in robotutil.py.
#  Workspace dimenions are assumed to be maximal (as defined in robotutil.py);
#  reads Arena_Coordinates.cfg for behavioral and housing module coordinates and dimensions (Alternatively uses in-script defined variables).
#  Example includes moving flies from housing module well number startWell up until endWell into behavior module arena number startArn until endArn.
#  Arena sequential order is determined by line-wise order of coordinates in Arena_Coordinates.cfg OR from list-wise highest to lowest X and Y coordinates as determined in-script (X and Y coordinates at the same order are considered a match in the latter case).



## Dependencies
import cv2
import numpy as np
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
from datetime import datetime
import commonFlyTasks as cft
from WorkspaceModules import FlyPlate
from WorkspaceModules import SocialArena

#### BEGIN PGM ####
coordfromcfg = False       # set True to read Arena_Coordinates.cfg file for coordinates
robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

fp = FlyPlate.FlyPlate(np.array([43.6, 91.1]),np.array([142.3, 27.4]))
arena = SocialArena.SocialArena(349.5, 269.3)
## Starts main transportation routine

cft.homeWithdraw(robot, fp, 1, refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=45)		# withdraws fly from individual well n
cft.arenaDeposit(robot, arena, 1, arenaRad=arena.Radii, turnZ=arena.POIz, airPos=300, airZ=arena.Vacz, closePos=180)		# deposits fly into arena n