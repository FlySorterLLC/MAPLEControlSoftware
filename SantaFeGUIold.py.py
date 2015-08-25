#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

import cv2
import numpy as np
import time
import ConfigParser

import robotutil

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

robot = robotutil.santaFe(CamIndex)

if robot.isInitialized == False:
    print "Initialization error."
    exit()
else:
    print "Robot initialized."


key = -1

# Move to lid
robot.moveTo( ( 425., 125., 0, 0, 0 ) )
robot.moveTo( ( 425., 125., 69.5, 0, 0) )
"""# Pick lid
robot.vacuum(True)
time.sleep(1)
robot.vacuum(False)
# Move lid up
robot.moveTo( ( 425., 125., 40.0, 0, 0) )
# Move to fly
robot.moveTo( ( 525., 50., 40.0, 0, 0 ) )

robot.moveTo( ( 525., 50., 40.0, 0, 29.75 ) )
robot.moveTo( ( 525., 50., 40.0, 0, 29.75 ) )

# Pick fly
# (Vac is already on)
time.sleep(1)

# Move fly up
robot.moveTo( ( 525.0, 50.0, 40.0, 0, 0 ) )

# Move to chamber

# Deposit fly

# Deposit lid

# Move out of the way
#robot.vacuum(False)
print "Done, waiting"

#while True:
    #time.sleep(1)
"""
cv2.destroyAllWindows()
robot.release()
# writeConfig(config)
