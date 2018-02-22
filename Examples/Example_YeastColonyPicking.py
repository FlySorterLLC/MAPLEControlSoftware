##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: Example_YeastColonyPicking.py
#  Description:

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
import commonYeastTasks as cyt
# Import relevant workspace
import ExampleYeastWorkspace

#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()


## Starts main yeast colony manipulation routine

curApp=0
cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1, colonyX=140+colonyX/100, colonyY=140+colonyY/100, skipAnchor=False, agarZ=39)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)

# begin writing routine
cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0,adjZ=2)
for i in range(0,17):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0, colonyX=curX+8, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#robot.home()

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1, colonyX=130+colonyX/100, colonyY=140+colonyY/100, skipAnchor=False, agarZ=39)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0,adjZ=2)
for i in range(17,17+18):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0, colonyX=curX+8, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#robot.home()

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1, colonyX=140-colonyX/100, colonyY=140-colonyX/100, skipAnchor=False, agarZ=39)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0,adjZ=2)
for i in range(17+18,17+18+16):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0, colonyX=curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#robot.home()

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1, colonyX=145+colonyX/100, colonyY=140+colonyY/100, skipAnchor=False, agarZ=39)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0,adjZ=2)
for i in range(17+18+16,17+18+16+9):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0, colonyX=curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#robot.home()

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 1, colonyX=150-colonyX/100, colonyY=140-colonyY/100, skipAnchor=False, agarZ=39)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],1)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0,adjZ=2)
for i in range(17+18+16+9,17+18+16+9+15):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0, colonyX=curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
#

# begin streaking routine
cyt.streakColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'], 0,  agarZ=38.8)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena'],0)
#

# discard last used applicator
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#

robot.home()