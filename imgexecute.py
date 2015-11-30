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
import ConfigParser

import robotutil
import imgprocess

## Some defaults & settings

# Configuration defaults
configDefaults = {'CamIndex': '0',
                  'OutputDir': 'C:/Users/DaveZ/Documents/Training Data',
                  }

# Read in the config, and assign values to the appropriate vars
def readConfig(config):
    global CamIndex, OutputDir
    config.read('SantaFe.cfg')
    CamIndex = config.getint('DEFAULT', 'CamIndex')
    OutputDir = config.get('DEFAULT', 'OutputDir')
    
# Write out the config file after transferring values from vars
def writeConfig(config):
    global FieldCamIndex, CloseCamIndex
    config.set('DEFAULT', 'CamIndex', CamIndex)
    config.set('DEFAULT', 'OutputDir', OutputDir)
    with open('FlySorter.cfg', 'wb') as configfile:
        config.write(configfile)

        
#### BEGIN PGM ####

# Open and read in configuration file
config = ConfigParser.RawConfigParser(configDefaults)
readConfig(config)

homeBool = 1

robot = robotutil.santaFe(CamIndex,homeBool)

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."
    
webcam = cv2.VideoCapture(2)
webcam.read()

close = cv2.VideoCapture(0)

close.set(cv2.cv.CV_CAP_PROP_EXPOSURE, -4)
close.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, 10)
close.set(cv2.cv.CV_CAP_PROP_GAIN, 5)
close.set(cv2.cv.CV_CAP_PROP_SATURATION, 12)
close.set(cv2.cv.CV_CAP_PROP_HUE, 13)
close.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 2592)
close.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1944)

# --------- Start Test Code -------

#move robot to hard coded fly pad location
reference = (390, 410)
PPMM = 28
padLocation = (1000, 3000, 0, 0, 0)     #location of top left hand corner of pad
padSize = (500, 500)                    #size of the pad in x, y
mazeLocation = (3000, 3000, 0, 0, 0)
imageSize = (1900/PPMM, 1900/PPMM)      #(1900, 1900) in pixels

def getAllFlies(self, robot, padLocation, padSize, imageSize, mazeLocation):
    
    duration = 2
    plateBool = 1
    a = imageProcess()
    
    #how many images do we need to take?
    xSweeps = padSize[0]/imageSize[0] + 1
    ySweeps = padSize[1]/imageSize[1] + 1
    
    #all the different locations we will need to image
    camXY = []
    for x in range(xSweeps+1):
        for y in range(ySweeps+1):
            camXY.append((x * padSize[0]/xSweeps, y * padSize[1]/ySweeps))
    
    for (x,y) in camXY:
        
        self.moveTo(padLocation[0] + x, padLocation[1] + y, 0, 0, 0)
        print "Imaging:", (x,y)
        s, img = close.read()
        targets = a.execute(img, reference)

        for (x,y) in targets:
            pt = (reference[0] + x, reference[1] + y, 0, 0, 0)
            print "Getting Fly at point:", pt
            self.moveTo(pt)
            self.dipAndGetFly(pt, duration, plateBool)

            self.moveTo(mazeLocation)      #drop them all off at a single location for now
            self.depositInMaze(mazeLocation, duration)

a = imageProcess()
a.getAllFlies(robot, padLocation, padSize, imageSize, mazeLocation)    
robot.release()