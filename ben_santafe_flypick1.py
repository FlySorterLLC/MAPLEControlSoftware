#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

import cv2
import numpy as np
import time
import ConfigParser

import ben_robotutil


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

homeBool=0

robot = ben_robotutil.santaFe(CamIndex,homeBool)

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."


key = -1

zPlane=43
hover=10
padZOffset=2
mazeZOffset=6
plateZOffset=11
padHover=zPlane-(padZOffset+hover)
mazeHover=zPlane-(mazeZOffset+hover)
plateHover=zPlane-(plateZOffset+hover)
well1=[345, 71]
maze1=[410.5, 145.5]

newY=well1[1]+(5)*9.2
robot.moveTo( ( well1[0]+80, newY-80, 0, 0, plateHover) )
robot.moveTo( ( maze1[0], maze1[1], 0, 0, plateHover) )
robot.moveTo( ( maze1[0], maze1[1], 0, 0, mazeHover) )
robot.suckFromMaze( (  maze1[0], maze1[1], 0, 0, mazeHover), 5)
robot.moveTo( ( well1[0]+80, newY-80, 0, 0, plateHover) )

##newY=well1[1]+(5)*9.2
##robot.moveTo( ( well1[0]+80, newY-80, 0, 0, plateHover) )
##robot.moveTo( ( well1[0], newY, 0, 0, plateHover) )
##robot.dipAndGetFly( ( well1[0], newY, 0, 0, plateHover), 2, 1)
##robot.moveTo( ( maze1[0], maze1[1], 0, 0, plateHover) )
##robot.moveTo( ( maze1[0], maze1[1], 0, 0, mazeHover) )
##robot.depositInMaze( (  maze1[0], maze1[1], 0, 0, mazeHover), 0.3)
##robot.moveTo( ( maze1[0], maze1[1], 0, 0, plateHover) )
##robot.moveTo( ( well1[0]+80, newY-80, 0, 0, plateHover) )

##robot.dipAndGetFly( ( well1[0], newY, 0, 0, plateHover), 2, 1)
##robot.dipAndDropFly( ( well1[0], newY, 0, 0, plateHover), 0.3)




##for i in range(0,8):
##    print 'i =', i+3
##    newY=well1[1]+(i+3)*9.2
##    robot.moveTo( ( well1[0], newY, 0, 0, plateHover) )
##    robot.dipAndGetFly( ( well1[0], newY, 0, 0, plateHover), 2, 1)
##    robot.dipAndDropFly( ( well1[0], newY, 0, 0, plateHover), 0.3)

##robot.dipAndGetFly( ( well1[0], well1[1], 0, 0, plateHover), 2)
##robot.moveTo( ( well1[0], well1[1], 0, 0, plateHover+hover) )
##robot.detachPuff(0)
##robot.moveTo( ( well1[0], well1[1], 0, 0, plateHover) )

print "Done, waiting"

while True:
    time.sleep(1)

cv2.destroyAllWindows()
robot.release()
# writeConfig(config)

