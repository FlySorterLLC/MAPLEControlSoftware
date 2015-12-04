#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Hardcoded program to pick & deposit flies

import cv2
import numpy as np
import time
import robotutil

import Workspace1


#### BEGIN PGM ####
robot = robotutil.santaFe("FlySorter.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

robot.home()

pointList = np.array( [ [100, 100],
                        [100, 101],
                        [100, 102],
                        [100, 103],
                        [100, 104],
                        [100, 105],
                        [101, 105],
                        [102, 105],
                        [103, 105],
                        [104, 105],
                        [105, 105],
                        [104, 104],
                        [103, 103],
                        [102, 102],
                        [101, 101],
                        [100, 100]] )

robot.moveXY(Workspace1.Workspace1['plate1'].getWell(0))
time.sleep(4)
robot.moveXY(Workspace1.Workspace1['plate1'].getWell(45))
time.sleep(4)
robot.moveXY(Workspace1.Workspace1['plate1'].getWell(95))
time.sleep(4)

robot.moveXYList(pointList)
    
robot.release()

