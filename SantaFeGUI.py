#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

import cv2
import numpy as np
import time
import robotutil

# And pass in the ZAxisBaseAddress here
robot = robotutil.santaFe("FlySorter.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

robot.home()

cv2.namedWindow("SantaFe")
img = robot.captureImage()
cv2.imshow("SantaFe", cv2.resize(img, (960,540)))

key = -1
while ( key != 27 ): # ESC to exit
    img = robot.captureImage()
    cv2.imshow("SantaFe", cv2.resize(img, (960,540)))
    key = cv2.waitKey(5)
    if  ( key == ord('h') ):
        robot.home()
    elif( key == ord('a') ):
        robot.moveRel(np.array([10.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('d') ):
        robot.moveRel(np.array([-10.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('w') ):
        robot.moveRel(np.array([0., 10.0, 0.0, 0.0, 0.0]))
    elif( key == ord('s') ):
        robot.moveRel(np.array([0, -10.0, 0.0, 0.0, 0.0]))

cv2.destroyAllWindows()
robot.release()
