##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

import cv2
import numpy as np
import time
import robotutil
import os.path

configFile = "MAPLE.cfg"

imgSize = ( 864, 648 )
crosshairPts = (( 422, 324 ), ( 442, 324 ), ( 432, 314 ), ( 432, 334 ))

ref = (420, 330)
PPMM = 1
mazeLoc = (65, 120, 0, 30, 30)

# Print a string on an image from the current position
def printPosition(robot, img):
    global imgSize, crosshairPts
    currentPosition = robot.getCurrentPosition()
    posStr = "X: {0[0]:.1f} Y:{0[1]:.1f} Z0: {0[2]:.2f} Z1: {0[3]:.2f} Z2: {0[4]:.2f}".format(currentPosition)
    cv2.rectangle(img, (0, imgSize[1]), (imgSize[0], imgSize[1]-30), (25, 25, 25), -1)
    cv2.putText(img, posStr, (25, imgSize[1]-10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
    cv2.putText(img, "? for help", (25, 30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255,255), 2)
    cv2.line(img, crosshairPts[0], crosshairPts[1], (0,0,0), 3)
    cv2.line(img, crosshairPts[2], crosshairPts[3], (0,0,0), 3)
    cv2.line(img, crosshairPts[0], crosshairPts[1], (255, 255, 255), 1)
    cv2.line(img, crosshairPts[2], crosshairPts[3], (255, 255, 255), 1)

# And pass in the ZAxisBaseAddress here
robot = robotutil.MAPLE(configFile)

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

# robot.home()

robot.light(True)

cv2.namedWindow("MAPLE")

imageMode = True
key = -1
img = cv2.resize(robot.captureImage(), imgSize)
startTime = time.time()
while ( key != 27 ): # ESC to exit
    if ( imageMode == True):
        # Update the image once a second
        if ( time.time() - startTime > 0.5 ):
            # Capture image, resize for the screen667u, and draw crosshairs
            img = cv2.resize(robot.captureImage(), imgSize)
            startTime = time.time()
    # Update the position and show image every time, though
    printPosition(robot, img)
    cv2.imshow("MAPLE", img)
    key = cv2.waitKey(5)
    if  ( key == ord('?') ):
        # Print help message
        print """Keyboard commands:
    ESC         - exit
    h           - home robot
    g           - go to coordinates
    p           - print coordinates (in this window)
    SPACE       - update the image
    m           - toggle between capturing images continuously and not
    c 			- Capture image and save

Move +/- 10mm:
    a/d         - X
    w/s         - Y
    u/j         - Z0
    i/k         - Z1
    o/l         - Z2

Modifier keys:
    SHIFT       - Move 1mm instead
    CTRL        - Move 0.1mm instead"""
    elif( key == ord('h') ):
        robot.home()
    elif( key == ord('g') ):
        inputStr = raw_input("Type coordinates, separated by spaces:\n")
        coords = inputStr.split(' ')
        if len(coords) != 5:
            print "Error: could not parse input."
        else:
            newPosition = np.array([0., 0., 0., 0., 0.])
            i=0
            for coord in coords:
                newPosition[i] = float(coord)
                i = i+1
            robot.moveTo(newPosition)

    elif( key == ord('p') ):
        print robot.getCurrentPosition()
    elif( key == ord(' ') ):
        img = cv2.resize(robot.captureImage(), imgSize)
    elif( key == ord('m') ):
        imageMode = not imageMode
    elif( key == ord('a') ):
        robot.moveRel(np.array([10.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('d') ):
        robot.moveRel(np.array([-10.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('w') ):
        robot.moveRel(np.array([0., 10.0, 0.0, 0.0, 0.0]))
    elif( key == ord('s') ):
        robot.moveRel(np.array([0, -10.0, 0.0, 0.0, 0.0]))
    elif( key == ord('u') ):
        robot.moveRel(np.array([0.0, 0.0, -10.0, 0.0, 0.0]))
    elif( key == ord('j') ):
        robot.moveRel(np.array([0.0, 0.0, 10.0, 0.0, 0.0]))
    elif( key == ord('i') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, -10.0, 0.0]))
    elif( key == ord('k') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 10.0, 0.0]))
    elif( key == ord('o') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, -10.0]))
    elif( key == ord('l') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, 10.0]))
    elif( key == ord('A') ):
        robot.moveRel(np.array([1.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('D') ):
        robot.moveRel(np.array([-1.0, 0.0, 0.0, 0.0, 0.0]))
    elif( key == ord('W') ):
        robot.moveRel(np.array([0., 1.0, 0.0, 0.0, 0.0]))
    elif( key == ord('S') ):
        robot.moveRel(np.array([0, -1.0, 0.0, 0.0, 0.0]))
    elif( key == ord('U') ):
        robot.moveRel(np.array([0.0, 0.0, -1.0, 0.0, 0.0]))
    elif( key == ord('J') ):
        robot.moveRel(np.array([0.0, 0.0, 1.0, 0.0, 0.0]))
    elif( key == ord('I') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, -1.0, 0.0]))
    elif( key == ord('K') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 1.0, 0.0]))
    elif( key == ord('O') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, -1.0]))
    elif( key == ord('L') ):
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, 1.0]))
    elif( key == 1 ): # ctrl-a
        robot.moveRel(np.array([0.1, 0.0, 0.0, 0.0, 0.0]))
    elif( key == 4 ): # ctrl-d
        robot.moveRel(np.array([-0.1, 0.0, 0.0, 0.0, 0.0]))
    elif( key == 23 ): # ctrl-w
        robot.moveRel(np.array([0., 0.1, 0.0, 0.0, 0.0]))
    elif( key == 19 ): # ctrl-s
        robot.moveRel(np.array([0, -0.1, 0.0, 0.0, 0.0]))
    elif( key == 21 ): # ctrl-u
        robot.moveRel(np.array([0.0, 0.0, -0.1, 0.0, 0.0]))
    elif( key == 10 ): # ctrl-j
        robot.moveRel(np.array([0.0, 0.0, 0.1, 0.0, 0.0]))
    elif( key == 9 ): # ctrl-i
        robot.moveRel(np.array([0.0, 0.0, 0.0, -0.1, 0.0]))
    elif( key == 11 ): # ctrl-k
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.1, 0.0]))
    elif( key == 15 ): # ctrl-o
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, -0.1]))
    elif( key == 12 ): # ctrl-l
        robot.moveRel(np.array([0.0, 0.0, 0.0, 0.0, 0.1]))
    # Capture the current image and save it to a file img X.png
    elif( key == ord('c') ):
        img = robot.captureImage()
        i=0
        filename = "MAPLE-" + str(i) + ".png"
        while os.path.isfile(filename):
            i += 1
            filename = "MAPLE-" + str(i) + ".png"
        cv2.imwrite(filename, img)

    else:
        if (( key != -1 ) and ( key != 27) ):
            print "Unknown keypress:", key

cv2.destroyAllWindows()
robot.release()
