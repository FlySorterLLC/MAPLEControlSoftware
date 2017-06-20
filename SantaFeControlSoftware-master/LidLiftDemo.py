#! /usr/bin/env python

## Copyright (c) 2016, FlySorter, LLC

## Hardcoded program to rearrange lids (demo)

import cv2
import numpy as np
import time
import robotutil

import Workspace2

#### BEGIN PGM ####
robot = robotutil.santaFe("SantaFe.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

#robot.home()

# Calculate the points where we need to take images to find flies
Workspace2.Workspace2['pad1'].calculateRegionCoords(robot.FOV)

# Construct some helpful points

xyOffset = np.array([-2., -2.])

# 
padFocusHeight = robot.Z1FloorHeight - \
                 Workspace2.Workspace2['baseThickness'] - \
                 Workspace2.Workspace2['pad1'].Z1WorkingThickness
padFlyPickHeight = robot.Z1FloorHeight - \
                   Workspace2.Workspace2['baseThickness'] - \
                   Workspace2.Workspace2['pad1'].Z2WorkingThickness + \
                   robot.Z2Offset[4] 
mazeLidHeight = robot.Z1FloorHeight - \
                Workspace2.Workspace2['baseThickness'] - \
                Workspace2.Workspace2['maze1'].Z0WorkingThickness + \
                robot.Z0Offset[2] + 0.5 
mazeFlyHeight = robot.Z1FloorHeight - \
                Workspace2.Workspace2['baseThickness'] - \
                Workspace2.Workspace2['maze1'].Z2WorkingThickness + \
                robot.Z2Offset[4] + 0.5

print "Camera focused on pad height:", padFocusHeight
print "Fly pick from pad height:", padFlyPickHeight
print "Lid pick from maze height:", mazeLidHeight
print "Fly dropoff at maze height:", mazeFlyHeight


#
ZImagePad = np.array([0., 0., mazeLidHeight-25., padFocusHeight, padFlyPickHeight-10.])
ZLid = np.copy(ZImagePad)

ZLid[2] = mazeLidHeight

print "Image on pad Z coords:", ZImagePad
print "Lid Z coords:", ZLid


print "--------------------------"

# time.sleep(10)
robot.moveZ(ZImagePad)

mazeCounter = 1
#robot.light(True)

# Now loop through the maze positions
numMazes = Workspace2.Workspace2['maze1'].getNumMazes()+1
numMazes = 17

for n in range(2, numMazes):
    source = Workspace2.Workspace2['maze1'].getMaze(n) + robot.Z0Offset[0:2]
    robot.moveXY(source-xyOffset)
    robot.moveXY(source)
    robot.dwell(1)
    robot.smallPartManipVenturi(True)
    robot.moveZ(ZLid)
    time.sleep(0.5)
    robot.moveZ(ZImagePad)

    # Now replace the lid
    dest = Workspace2.Workspace2['maze1'].getMaze(n-1) + robot.Z0Offset[0:2]
    robot.moveXY(dest-xyOffset)
    robot.moveXY(dest)
    robot.dwell(1)
    robot.moveZ(ZLid)
    robot.smallPartManipAir(True)
    robot.smallPartManipVenturi(False)
    robot.moveZ(ZImagePad)
    robot.smallPartManipAir(False)
    time.sleep(0.1)
        

cv2.destroyAllWindows()
robot.release()

