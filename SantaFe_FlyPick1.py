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

robot.moveXY(Workspace1.Workspace1['plate1'].getWell(0))
time.sleep(4)
robot.moveXY(Workspace1.Workspace1['plate1'].getWell(45))
time.sleep(4)
robot.moveXY(Workspace1.Workspace1['plate1'].getWell(95))
    
robot.release()

