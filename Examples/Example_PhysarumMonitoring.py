##
#
#  File: Example_PhysarumMonitoring.py
#  Description: Repeatedly captures images of slime mold plates arranged in a 3 by 3 configuration for high-throughput time lapse movies and offline analysis.
#  Inter-image interval and total duration are variable.

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
import commonYeastTasks as cyt
import ExampleYeastWorkspace

#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

imgInterval = 60		# in seconds
tTotal = 0
picnum = 0
while tTotal < 60*60*10:		# in seconds
	tStart = time.time()
	for numArena in range(0,9):
		curImg = cyt.imgArena(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], numArena, light=0)
		time.sleep(0.1)
		cv2.imwrite(str(numArena)+ '_' + str(picnum) + '.png', curImg)
	tEnd = time.time()
	tDelta = tEnd - tStart
	tWait = imgInterval - tDelta
	tTotal = tTotal + tDelta + tWait
	picnum = picnum + 1
	print 'Waiting', tWait, 'seconds.'
	time.sleep(tWait)