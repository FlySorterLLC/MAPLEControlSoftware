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
    writeConfig(config)
    exit()
else:
    print "Robot initialized." #no timeouts up to here, z-axis 1 still timeout


key = -1

# Move to lid
#robot.moveTo( ( 25., 25., 0., 0, 0. ) )
#robot.moveTo( ( 425., 125., 0., 0, 0.) )
#robot.moveTo( ( 425., 125., 69.5, 0, 0) )

#Begin debugging of movement error
# X and Y work, no need to test

"""robot.moveTo( (0., 0., 60., 0., 0 ) )
robot.moveTo( (0., 0., 60., 0., 80., ) )"""


#starting mock procedure (intermediate movements can be combined)

robot.moveTo( (500., 0., 0., 0., 0 ) )
robot.vacuum(True)
robot.moveTo( (500., 0., 0., 0., 80 ) )
time.sleep(1)
robot.moveTo( (500., 0., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (586., 0., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (586., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (586., 55., 47., 0., 0 ) ) #Z1 higher up that Z3!
time.sleep(1)
robot.moveTo( (586., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (666., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (666., 55., 0., 0., 27 ) )
time.sleep(1)
robot.vacuum(False)
robot.moveTo( (666., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (540., 55., 0., 0., 0 ) )
time.sleep(1)
robot.vacuum(True)
robot.moveTo( (540., 55., 0., 0., 84 ) )
time.sleep(1)
robot.moveTo( (540., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (622., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (622., 55., 47., 0., 0 ) )
time.sleep(1)
robot.moveTo( (622., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (707., 55., 0., 0., 0 ) )
time.sleep(1)
robot.moveTo( (707., 55., 0., 0., 27 ) )
time.sleep(1)
robot.vacuum(False)

# Pick lid
#print "preparing to vacuum"
#robot.vacuum(True)
#time.sleep(5)
#robot.vacuum(False)

# Move lid up
#print "beginning part 2 of code"
#print "a"
#robot.moveTo( ( 425., 125., 40.0, 0, 0) )
# Move to fly
#print "b"
#robot.moveTo( ( 525., 50., 40.0, 0, 0 ) )
#print "c"
#robot.moveTo( ( 525., 50., 40.0, 0, 29.75 ) )
#robot.moveTo( ( 525., 50., 40.0, 0, 29.75 ) )
# Pick fly
# (Vac is already on)
time.sleep(1)

# Move fly up
#print "d"
#robot.moveTo( ( 525.0, 50.0, 40.0, 0, 0 ) )
# Move to chamber

# Deposit fly

# Deposit lid

# Move out of the way
#robot.vacuum(False)
print "Done, waiting"

#while True:
    #time.sleep(1)

cv2.destroyAllWindows()
robot.release()
writeConfig(config)
