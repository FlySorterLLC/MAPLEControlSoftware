##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

## Program to dispense flies & deposit into a FlyPlate

import cv2
import numpy as np
import time
import robotutil

import Workspace1

if Workspace1.Workspace1['dispenser1'].dispenserPort is None:
    print "Dispenser init error."
    exit()

#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

#robot.home()

# Construct some helpful points
xyOffset = np.array([-2.5, -2.5])

dispenserXY = Workspace1.Workspace1['dispenser1'].dispenserPoint + robot.Z2Offset[0:2]

transitHeight = robot.Z1FloorHeight + robot.Z2Offset[4] - \
                Workspace1.Workspace1['baseThickness'] - \
                Workspace1.Workspace1['plate1'].MaxThickness - 10

dispenserHeight = robot.Z1FloorHeight + robot.Z2Offset[4] - \
                  Workspace1.Workspace1['baseThickness'] - \
                  Workspace1.Workspace1['dispenser1'].Z2WorkingThickness

plateHeight = robot.Z1FloorHeight + robot.Z2Offset[4] - \
              Workspace1.Workspace1['baseThickness'] - \
              Workspace1.Workspace1['plate1'].Z2WorkingThickness


print "Transit height:", transitHeight
print "Fly pick from dispeser height:", dispenserHeight
print "Fly deposit in plate height:", plateHeight

transitZ = np.array([0., 0., 0., 0., transitHeight])
dispenserZ = np.array([0., 0., 0., 0., dispenserHeight])
plateZ = np.array([0., 0., 0., 0., plateHeight])

robot.light(True)

flyCount = 0

while flyCount < 96:

    # Move Z to transit height
    robot.dwell(1)
    robot.moveZ(transitZ)

    # Move XY to dispenser
    robot.moveXY(dispenserXY+xyOffset)
    robot.moveXY(dispenserXY)

    # Move Z to dispenser
    robot.dwell(1)
    robot.moveZ(dispenserZ)

    # Enable vacuum
    robot.dwell(1)
    robot.flyManipVac(True)

    # Dispense fly
    if ( Workspace1.Workspace1['dispenser1'].dispenseFly() == 1 ):
        print "Failed to dispense fly."
        robot.moveZ(transitZ)
        break

    # Move Z to transit height
    robot.moveZ(transitZ)

    # Move XY to next well
    plateXY = Workspace1.Workspace1['plate1'].getWell(flyCount) + robot.Z2Offset[0:2]
    robot.moveXY(plateXY + xyOffset)
    robot.moveXY(plateXY)

    # Move Z to deposity fly height
    robot.dwell(1)
    robot.moveZ(plateZ)

    # Release fly
    robot.flyManipVac(False)
    robot.flyManipAir(True)
    time.sleep(1)

    # Move Z to transit height
    robot.moveZ(transitZ)
    robot.flyManipAir(False)
    flyCount += 1


robot.light(False)

cv2.destroyAllWindows()
robot.release()
