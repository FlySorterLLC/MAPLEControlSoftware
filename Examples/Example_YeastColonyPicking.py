##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: Example_YeastColonyPicking.py
#  Description: Picks yeast colonies from source plate and deposits or streaks them into target plates. Lids are withdrawn and replaced before and after probing plates.
#  Applicators are replaced before a new source colony is picked. Source colonies are detected automatically. Colonies are deposited into target plates according to predetermined schema.

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
cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)	# loads first applicator into MAPLE's organism manipulator end effector
equippedApp = cyt.applicatorTest(robot)		# tests whether applicator is loaded
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2)	# returns viable source colony coordinates (in camera field of view)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)		# withdraws lid, tests whether lid has been withdrawn
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2, colonyX=309+colonyX, colonyY=28+colonyY, skipAnchor=False, agarZ=40)	# picks source colony
robot.homeZ2()	# withdraws yeast applicator
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)	# replaces lid onto source plate

# begin writing routine
cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5,adjZ=2.5)
for i in range(0,17):	# places picked colony onto target plate according to placement scheme (17 called locations make up the letter M)
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5, colonyX=35+curX+8, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)		# discards used applicator, tests whether applicator was discarded
	equippedApp = cyt.applicatorTest(robot)

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1 
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2, colonyX=309+colonyX, colonyY=28+colonyY, skipAnchor=False, agarZ=40)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5,adjZ=1.5)
for i in range(17,17+18):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5, colonyX=35+curX+8, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2, colonyX=309+colonyX, colonyY=28+colonyY, skipAnchor=False, agarZ=40)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5,adjZ=1.5)
for i in range(17+18,17+18+16):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5, colonyX=35+curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2, colonyX=309+colonyX, colonyY=28+colonyY, skipAnchor=False, agarZ=40)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5,adjZ=1.5)
for i in range(17+18+16,17+18+16+9):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5, colonyX=35+curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5)
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)

cyt.applicatorEquip(robot, ExampleYeastWorkspace.YeastWorkspace['yeastApplicatorPlate'], curApp)
equippedApp = cyt.applicatorTest(robot)
curApp = curApp+1
test = 1
colonyX, colonyY = cyt.detectColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2)
while test == 1:
	cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
	robot.dwell(10)
	test = cyt.lidTest(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)
cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 2, colonyX=309+colonyX, colonyY=28+colonyY, skipAnchor=False, agarZ=40)
robot.homeZ2()
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],2)

cyt.lidWithdraw(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5,adjZ=1.5)
for i in range(17+18+16+9,17+18+16+9+15):
	curX = cyt.getLogoCoord(i)[0]
	curY = cyt.getLogoCoord(i)[1]
	cyt.colonyProbe(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5, colonyX=35+curX+5.5, colonyY=curY, skipAnchor=True)
	time.sleep(0.1)
#

# begin streaking routine
cyt.streakColony(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], 5,  agarZ=38.8)		# streaks out last picked colony onto target plate
cyt.lidPlace(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'],5)
#

# discard last used applicator
while equippedApp ==1:
	cyt.applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=80)
	equippedApp = cyt.applicatorTest(robot)
#

robot.home()