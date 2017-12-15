##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
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
import remoteOperation as remote
# Import relevant workspace
import ExampleSocialWorkspace

#### BEGIN PGM ####
coordfromcfg = False       # set True to read Arena_Coordinates.cfg file for coordinates
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

## Starts main example transportation routines

#cft.homeWithdraw(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'], 1, refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=45)		# withdraws fly from individual well n
#cft.arenaDeposit(robot, ExampleSocialWorkspace.SocialWorkspace['socialArena'], 1, arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii, turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz, airPos=50, airZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz, closePos=0)
#cft.arenaWithdraw(robot, ExampleSocialWorkspace.SocialWorkspace['socialArena'], 1, arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii, turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz, vacPos=50, vacZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz, closePos=0, vacstrategy=2, vacBurst=1, imgshow=0)
#cft.homeDeposit(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'], 1, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44)
#cft.collectHatchedForT(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'], ExampleSocialWorkspace.SocialWorkspace['dispenser1'], onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=1, carryovernDispensed=0, collectT=20, collectInt=10, maxconsecstuck=4)
mail = remote.listenMode(statusURL="", duration=60, listenInterval=2, robotEMailAccount='SantaFailure@gmail.com', PWrobotEMailAccount='H@rvard2017!')
remote.doInstruct(robot=robot, instruction=mail['instruct'], mailfrom=mail['from'], values=mail['values'], arena=ExampleSocialWorkspace.SocialWorkspace['socialArena'], FlyPlate=ExampleSocialWorkspace.SocialWorkspace['plate1'], turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz, vacZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz, arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii, dispenser=ExampleSocialWorkspace.SocialWorkspace['dispenser1'], HomeZwd=44, HomeZdp=44)
