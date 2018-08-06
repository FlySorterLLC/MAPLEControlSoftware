##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

## Hardcoded program to pick & deposit flies

import cv2
import numpy as np
import time
import robotutil

import Workspace2

#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

#robot.home()

# Calculate the points where we need to take images to find flies
Workspace2.Workspace2['pad1'].calculateRegionCoords(robot.FOV)

# Construct some helpful points

xyOffset = np.array([-5., -5.])

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
ZGrabFly = np.copy(ZImagePad)
ZDepositFly = np.copy(ZImagePad)
ZLid = np.copy(ZImagePad)

ZGrabFly[4] = padFlyPickHeight
ZDepositFly[4] = mazeFlyHeight
ZLid[2] = mazeLidHeight

print "Image on pad Z coords:", ZImagePad
print "Pickup fly Z coords:", ZGrabFly
print "Deposite fly Z coords:", ZDepositFly
print "Lid Z coords:", ZLid


print "--------------------------"

# time.sleep(10)
robot.moveZ(ZImagePad)

mazeCounter = 1
robot.light(True)

# Now loop through the imaging points
for imgPt in Workspace2.Workspace2['pad1'].imagePoints:
    if (mazeCounter > Workspace2.Workspace2['maze1'].getNumMazes()):
        break
    print "Image point:", imgPt
    robot.moveXY(imgPt)
    robot.dwell(10)
    img = robot.captureImage()
    robot.dwell(100)
    flyPoint = robot.findFly(img)
    while (( flyPoint is not None ) and (mazeCounter <= Workspace2.Workspace2['maze1'].getNumMazes()) ):
        # flyPoint is actually an offset from the current position (imgPt)
        print "Found fly at:", flyPoint+imgPt
        ##cv2.imshow("flypad", cv2.resize(img, ( 864, 648 )) )
        ##time.sleep(2)
        # Grab fly
        flyXYPoint = imgPt + flyPoint + robot.Z2Offset[0:2]
        robot.moveXY(flyXYPoint-xyOffset)
        robot.moveXY(flyXYPoint)
        robot.dwell(1)
        robot.moveZ(ZGrabFly)
        robot.flyManipVac(True)
        time.sleep(0.5)
        robot.dwell(1)
        robot.moveZ(ZImagePad)

        # Move it to the next available maze
        # First, remove the lid

        mazePt = Workspace2.Workspace2['maze1'].getMaze(mazeCounter) + robot.Z0Offset[0:2]
        robot.moveXY(mazePt-xyOffset)
        robot.moveXY(mazePt)
        robot.dwell(1)
        robot.smallPartManipVac(True)
        robot.moveZ(ZLid)
        time.sleep(0.5)
        robot.dwell(1)
        robot.moveZ(ZImagePad)

        # Now deposit the fly
        mazePt = Workspace2.Workspace2['maze1'].getMaze(mazeCounter) + robot.Z2Offset[0:2]
        robot.moveXY(mazePt-xyOffset)
        robot.moveXY(mazePt)
        robot.dwell(1)
        robot.moveZ(ZDepositFly)
        robot.flyManipVac(False)
        robot.flyManipAir(True)
        time.sleep(0.25)
        robot.flyManipAir(False)
        time.sleep(0.15)
        robot.moveZ(ZImagePad)

        # Now replace the lid
        mazePt = Workspace2.Workspace2['maze1'].getMaze(mazeCounter) + robot.Z0Offset[0:2]
        robot.moveXY(mazePt-xyOffset)
        robot.moveXY(mazePt)
        robot.dwell(1)
        robot.moveZ(ZLid)
        robot.smallPartManipVac(False)
        robot.smallPartManipAir(True)
        robot.moveZ(ZImagePad)
        robot.smallPartManipAir(False)

        mazeCounter += 1
        # Take another image in case there are more flies in this region
        robot.moveZ(ZImagePad)
        print "Image point:", imgPt
        robot.moveXY(imgPt)
        robot.dwell(10)
        img = robot.captureImage()
        flyPoint = robot.findFly(img)


cv2.destroyAllWindows()
robot.release()
