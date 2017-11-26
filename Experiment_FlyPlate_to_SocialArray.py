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
else:       # Rows denote y position changes in arena, columns denote x changes
    nrows = 9       # maximum possible rows and columns in arena
    ncols = 9
    nhrows = 12
    nhcols = 8
    rowstart = 269.3      # Y coord of first point of interest (POI)
    colstart = 349.5      # X coord of first POI
    hrowstart = 245.2     # Coords of first home POI
    hcolstart = 496.5
    camdiffx = 44       # difference in cam and manip coords
    camdiffy = 6.3 
    camsharpz = 40      # position at which POIs are sharp
    xdif = 32.1       # x difference between POIs
    ydif = 32       # y difference between POIs
    hxdif = 9       # same for home POIs
    hydif = 9      
    POIrad = 10.5     # radius of arena POI opening
    POIz = 49
    Vacz = 50       # depth from which to vacuum the fly out of the POI
    homezdepos = 43      # depth at which flies can be deposited in home
    homezwd = 44        # depth at which flies can be vacuumed out of home
    dispx = 639.5
    dispy = 113
    dispz = 33


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



## Move vector
flyplatevect = 0,8,16,24,32,40,48,56,64,72
deposvect = 38,38,39,39,40,40,41,41,42,42
schemevect = 0,4,1,6,2,7,3,8,5,9
## Main Loop
for n in range(0,9,2):
    print 'Fly ',schemevect[n] + 1, ' into compartment ', n + 1 
    robot.homeWithdraw(homecoordX=FlyHomeX[flyplatevect[schemevect[n]]], homecoordY=FlyHomeY[flyplatevect[schemevect[n]]], refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, vacDur=4000, homeZ=homezwd)
    robot.arenaDeposit(camcoordX=CamX[deposvect[n]], camcoordY=CamY[deposvect[n]], camcoordZ=camsharpz, arenacoordX=ManipX[deposvect[n]], arenacoordY=ManipY[deposvect[n]], arenaRad=POIrad, turnZ=POIz, airPos=55, airZ=Vacz, closePos=1, airBurst=1, imgshow=0)
robot.home()
# robot.homeDeposit(homecoordX=FlyHomeX[flyplatevect[n]], homecoordY=FlyHomeY[flyplatevect[n]], refptX='N', refptY='N', carefulZ=7, vacBurst=1, homeZ=homezwd)