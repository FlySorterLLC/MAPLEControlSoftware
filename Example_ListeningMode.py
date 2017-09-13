## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: Example_ListeningMode.py
#  Description: Example routine calling highest-level functions in robotutil.py.
#  Send an instruction via email using the apropriate format (see robotutil.py for example) to start, e.g., virgining routine remotely.
#  Workspace dimenions are assumed to be maximal (as defined in robotutil.py);
#  reads Arena_Coordinates.cfg for behavioral and housing module coordinates and dimensions (Alternatively uses in-script defined variables).
#  Arena sequential order is determined by line-wise order of coordinates in Arena_Coordinates.cfg OR from list-wise highest to lowest X and Y coordinates as determined in-script (X and Y coordinates at the same order are considered a match in the latter case).



## Dependencies
import cv2
import numpy as np
import time
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
from datetime import datetime

#### BEGIN PGM ####
coordfromcfg = False       # Set True to read Arena_Coordinates.cfg file for coordinates
robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

## Reads Arena_Coordinates.cfg file to pull coordinates. Refer to file for formatting. Supports altered coordinate-ordering and uneven spacing.
if coordfromcfg == True:
    print "Reading arena camera and manipulator coordinates...",
    config = ConfigParser.RawConfigParser()
    config.read("Arena_Coordinates.cfg")
    CamX = config.get('ARENA', 'XCam')
    CamY = config.get('ARENA', 'YCam')
    ManipX = config.get('ARENA', 'XManip')
    ManipY = config.get('ARENA', 'YManip')
    Radii = config.get('ARENA', 'Radii')
    FlyHomeX = config.get('HOME', 'XHome')
    FlyHomeY = config.get('HOME', 'YHome')

    print 'Camera X coordinates are:', CamX.splitlines()
    CamX = CamX.splitlines()
    print 'Camera Y coordinates are:', CamY.splitlines()
    CamY = CamY.splitlines()
    print 'Manipulator X coordinates are:', ManipX.splitlines()
    ManipX = ManipX.splitlines()
    print 'Manipulator Y coordinates are:', ManipY.splitlines()
    ManipY = ManipY.splitlines()
    print 'Radii coordinates are:', Radii.splitlines()
    Radii = Radii.splitlines()
    print 'FlyHomeX coordinates are:', FlyHomeX.splitlines()
    FlyHomeX = FlyHomeX.splitlines()
    print 'FlyHomeY coordinates are:', FlyHomeY.splitlines()
    FlyHomeY = FlyHomeY.splitlines()
    print "done."

    ## Alternative coordinate input; assumes grid layout and even spacing between arenas
else:       # rows denote y position changes in arena, columns denote x changes (nomenclature)
    nrows = 9       # maximum possible rows and columns of arenas in behavioral module
    ncols = 9
    nhrows = 12     # maximum possible rows and columns of individual wells in housing module
    nhcols = 8
    rowstart = 269.3      # Y coord of first point of interest (POI) NOTE: POI refers to arena or individual well with numerically highest X and Y coordinates.
    colstart = 349.5      # X coord of first POI
    hrowstart = 245.2     # Y coords of first home POI
    hcolstart = 496.5     # X coords of first home POI
    camdiffx = 44       # difference cam and manip coords (constant camera offset; all coordinates in reference to fly handling end effector)
    camdiffy = 6.3 
    camsharpz = 40      # depth at which POIs are sharp (depending on focal length etc)
    xdif = 32.1       # x difference between POIs (how far apart arena mid points are)
    ydif = 32       # y difference between POIs
    hxdif = 9       # same for home POIs
    hydif = 9      
    POIrad = 10.5     # radius of arena POI opening
    POIz = 49       # depth at which arenas may be accessed by end effector
    Vacz = 50       # depth from which to vacuum the fly out of arena
    homezdepos = 43      # depth at which flies can be deposited in home
    homezwd = 44        # depth at which flies can be vacuumed out of home
    dispx = 639.5       # for visualization only; maximum workspace length in coordinates
    dispy = 113
    dispz = 33
    listeningTime = 300     # cycles of repeat listening mode (Note: Robot is unresponsive to manual commands while listening). Corresponds to elapsed minutes if no instruction received.

    ## Computes arena and well positions (assumes grid-layout and assigns order according to descending numerical coordinate values)
    arncoordsX = np.zeros((nrows, ncols))
    arncoordsY = np.zeros((nrows, ncols))
    curcoord = range(2)
    for arnrow in range(0,nrows):
        for arncol in range(0,ncols):
            curcoord[0] = colstart - (arncol * xdif)
            curcoord[1] = float(rowstart) - float(arnrow * ydif)
            arncoordsX[arnrow, arncol] = curcoord[0]
            arncoordsY[arnrow, arncol] = float(curcoord[1])
    hcoordsX = np.zeros((nhrows, nhcols))
    hcoordsY = np.zeros((nhrows, nhcols))
    for hrow in range(0,nhrows):
        for hcol in range(0,nhcols):
            curcoord[0] = hcolstart - (hcol * hxdif)
            curcoord[1] = float(hrowstart) - float(hrow * hydif)
            hcoordsX[hrow, hcol] = curcoord[0]
            hcoordsY[hrow, hcol] = float(curcoord[1])
    ManipX = np.reshape(arncoordsX, (nrows*ncols,1))
    ManipY = np.reshape(arncoordsY, (nrows*ncols,1))
    CamX = ManipX - camdiffx
    CamY = ManipY - camdiffy
    FlyHomeX = np.reshape(hcoordsX, (nhrows*nhcols,1))
    FlyHomeY = np.reshape(hcoordsY, (nhrows*nhcols,1))
    Radii = (ManipX / ManipX) * POIrad

## Starts main listening mode routine
timeelapsed = 0
while ( timeelapsed <= listeningTime ):
    mail = robot.listenMode()       # engages listening mode for 1 minute
    if mail != None:        # performs instruction
        robot.home()
        robot.doInstruct(instruction=mail['instruct'], mailfrom=mail['from'], values=mail['values'], CamX=CamX, CamY=CamY, CamZ=camsharpz, ManipX=ManipX, ManipY=ManipY, turnZ=POIz, vacZ=Vacz, arenaRad=10.5, HomeX=FlyHomeX, HomeY=FlyHomeY, HomeZwd=homezwd, HomeZdp=homezdepos)
        robot.home()
    timeelapsed = timeelapsed + 1       # in minutes
    print 'time since start:', timeelapsed