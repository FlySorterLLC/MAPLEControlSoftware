#! /usr/bin/env python

## Copyright (c) 2014, FlySorter, LLC

import cv2
import numpy as np
import ConfigParser

import robotutil

## Some defaults & settings

# Configuration defaults
configDefaults = {'COM': '2',
                  'CamIndex': '2',
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

# Close up shop, write config
cv2.destroyAllWindows()
robot.release()
writeConfig(config)
