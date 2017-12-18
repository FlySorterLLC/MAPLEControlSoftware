##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: Example_VirginCollection.py
#  Description: Example routine calling highest-level functions in robotutil.py.
#  Workspace dimenions are imported from ExampleSocialWorkspace.
#  Collets newly hatched flies every intCollect seconds for a total of tCollect seconds.
#  Newly hatched flies are sequentially deposited into FlyPlate wells.

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
import commonFlyTasks as cft

# Import relevant workspace
import ExampleSocialWorkspace

# Virgin collection variables
tCollect = 86400		# Total virgin collection time in seconds.
intCollect = 1800		# Virgin collection interval in seconds.

#### Initialize MAPLE ####
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

## Starts main virgin collection routine
cft.collectHatchedForT(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'], ExampleSocialWorkspace.SocialWorkspace['dispenser1'], onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=1, carryovernDispensed=0, collectT=tCollect, collectInt=intCollect, maxconsecstuck=4)
