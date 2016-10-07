#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC
## Code that combines robotutil and imgprocess to find flies and execute commands 
## getAllFlies takes as primary input the location and size of a maze and of a fly pad
## and automatically takes all flys in the fly pad area and moves them to the maze location
## TODO: hard code/use machine vision to specify actual locations of mazes

## By: Will Long
## MRU: Nov 29 2015

import cv2
import numpy as np
import time

import robotutil
import imgprocess

## Some defaults & settings

        
#### BEGIN PGM ####

robot = robotutil.santaFe("FlySorter.cfg")

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."

robot.home()

# --------- Start Test Code -------

#move robot to hard coded fly pad location
reference = (390, 410)
PPMM = 28
padLocation = (110, 110, 10, 0, 0)     #location of top left hand corner of pad
padSize = (50, 50)                    #size of the pad in x, y
mazeLocation = (500, 100, 10, 0, 0)
imageSize = (1900/PPMM, 1900/PPMM)      #(1900, 1900) in pixels

def generateMazeLocs():
    mazeLocs = []
    oddMaze1 = (304, 233)
    evenMaze1 = (290, 204)

    for row in range(5):        # camera can't seem to capture farthest row
        mazeRow = []
        for col in range(9):
            mazeRow.append((oddMaze1[0] - col * 30, oddMaze1[1] - row * 56))
        mazeLocs.append(mazeRow)

    return mazeLocs

def getAllFlies(robot, padLocation, padSize, imageSize, mazeLocation):
    
    duration = 2
    plateBool = 1
    a = imgprocess.imageProcess()
    
    #how many images do we need to take?
    xSweeps = padSize[0]/imageSize[0] + 1
    ySweeps = padSize[1]/imageSize[1] + 1
    
    #all the different locations we will need to image
    camXY = []
    for x in range(xSweeps+1):
        for y in range(ySweeps+1):
            camXY.append((x * padSize[0]/xSweeps, y * padSize[1]/ySweeps))
    
    for (x,y) in camXY:
        
        robot.moveTo((padLocation[0] + x, padLocation[1] + y, padLocation[2], padLocation[3], padLocation[4]))
        print "Imaging:", (x,y)
        img = robot.captureImage()
        cv2.imwrite('actual.bmp', img)
        targets = a.execute(img, reference)

        for (x,y) in targets:
            pt = (reference[0] + x, reference[1] + y, padLocation[2], padLocation[3], padLocation[4])
            print "Getting Fly at point:", pt
            robot.moveTo(pt)
            robot.dipAndGetFly(pt, duration, plateBool)

            robot.moveTo(mazeLocation)      #drop them all off at a single location for now
            robot.depositInMaze(mazeLocation, duration)

robot.light(True)
# getAllFlies(robot, padLocation, padSize, imageSize, mazeLocation)  
mazes = generateMazeLocs() 
print mazes 
for row in range(6):
    img = cv2.resize(robot.captureImage(), ( 864, 648 ))
    cv2.imshow("camera", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    for maze in mazes[row]:
        robot.moveTo((maze[0], maze[1], 0, 30, 0))

robot.release()
