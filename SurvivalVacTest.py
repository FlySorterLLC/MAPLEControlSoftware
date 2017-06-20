import cv2
import numpy as np
import time
import robotutil
#import Workspace1
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



# FlyPOSfrom = range(0,48)
# FlyPOSto = range(0,48)
# instructmage = robot.visualizeInstruct(HXcoords=FlyHomeX, HYcoords=FlyHomeY, Xcoords=ManipX, Ycoords=ManipY, ArenaSide='R', POIrad=POIrad, dispx=dispx, dispy=dispy, instHN=FlyPOSfrom, instN=FlyPOSto, instdispN=0, start='home')
# for n in range(len(FlyPOSfrom)):
#     robot.updateCurInstruct(background=instructmage, N=n, HXcoords=FlyHomeX, ArenaSide='R', HYcoords=FlyHomeY, Xcoords=ManipX, Ycoords=ManipY, dispx=dispx, dispy=dispy, instHN=FlyPOSfrom, instN=FlyPOSto, instdispN=0, start='home')
#     Fail = robot.homeWithdraw(homecoordX=FlyHomeX[FlyPOSfrom[n]], homecoordY=FlyHomeY[FlyPOSfrom[n]], refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=homezwd)
#     instructmage = robot.updatePastInstruct(background=instructmage, N=n, HXcoords=FlyHomeX, ArenaSide='R', HYcoords=FlyHomeY, Xcoords=ManipX, Ycoords=ManipY, POIrad=POIrad, dispx=dispx, dispy=dispy, instHN=FlyPOSfrom, instN=FlyPOSto, instdispN=0, start='home', updateWhich='home', Fail=Fail['limit'])
#     robot.updateCurInstruct(background=instructmage, N=n, HXcoords=FlyHomeX, ArenaSide='R', HYcoords=FlyHomeY, Xcoords=ManipX, Ycoords=ManipY, dispx=dispx, dispy=dispy, instHN=FlyPOSfrom, instN=FlyPOSto, instdispN=0, start='home')
#     Fail = robot.arenaDeposit(camcoordX=CamX[FlyPOSto[n]], camcoordY=CamY[FlyPOSto[n]], camcoordZ=camsharpz, arenacoordX=ManipX[FlyPOSto[n]], arenacoordY=ManipY[FlyPOSto[n]], arenaRad=POIrad, turnZ=POIz, airPos=300, airZ=Vacz, closePos=180)
#     instructmage = robot.updatePastInstruct(background=instructmage, N=n, HXcoords=FlyHomeX, ArenaSide='R', HYcoords=FlyHomeY, Xcoords=ManipX, Ycoords=ManipY, POIrad=POIrad, dispx=dispx, dispy=dispy, instHN=FlyPOSfrom, instN=FlyPOSto, instdispN=0, start='home', updateWhich='arena', Fail=Fail['miss'])
# filename = str(datetime.now())
# filename = filename[0:10] + '.png'
# cv2.imshow('Final', instructmage)
# cv2.imwrite(filename, instructmage)
# cv2.waitKey(10)

timeelapsed = 0
while ( timeelapsed <= 300 ):
    mail = robot.listenMode()
    if mail != None:
        robot.home()
        robot.doInstruct(instruction=mail['instruct'], mailfrom=mail['from'], values=mail['values'], CamX=CamX, CamY=CamY, CamZ=camsharpz, ManipX=ManipX, ManipY=ManipY, turnZ=POIz, vacZ=Vacz, arenaRad=10.5, HomeX=FlyHomeX, HomeY=FlyHomeY, HomeZwd=homezwd, HomeZdp=homezdepos)
        robot.home()
    timeelapsed = timeelapsed + 1       # in minutes
    print 'time since start:', timeelapsed

#for i in range(mail['values'][0],mail['values'][1],1):
    #robot.arenaWithdraw(camcoordX=CamX[i], camcoordY=CamY[i], camcoordZ=camsharpz, arenacoordX=ManipX[i], arenacoordY=ManipY[i], arenaRad=Radii[i], turnZ=POIz, vacPos=50, vacZ=Vacz, closePos=180, vacBurst=1, strategy=2)
    #robot.dwell(10)
    #robot.homeDeposit(homecoordX=FlyHomeX[i], homecoordY=FlyHomeY[i], carefulZ=5, vacBurst=1, homeZ=44)
    #robot.dwell(10)
#for j in range(0,3):
    #flyremainvect = robot.sweep(CamX[mail['values'][0]:mail['values'][1]], CamY[mail['values'][0]:mail['values'][1]], camz=camsharpz)
    #for i in range(0,len(flyremainvect),1):
        #robot.arenaWithdraw(camcoordX=CamX[flyremainvect[i]], camcoordY=CamY[flyremainvect[215223i]], camcoordZ=camsharpz, arenacoordX=ManipX[flyremainvect[i]], arenacoordY=ManipY[flyremainvect[i]], arenaRad=Radii[flyremainvect[i]], turnZ=POIz, vacPos=50, vacZ=Vacz, closePos=180, vacBurst=1, strategy=2)
        #robot.dwell(10)
        #robot.homeDeposit(homecoordX=FlyHomeX[flyremainvect[i]], homecoordY=FlyHomeY[flyremainvect[i]], carefulZ=5, vacBurst=1, homeZ=44)
        #robot.dwell(10)

#print 'Notifying user of failed arenas'
#unsurevect = robot.sweep(CamX[flyremainvect], CamY[flyremainvect], camz=camsharpz)
#try:
    #flyremainvect = flyremainvect[unsurevect]
    #robot.SaveArenaPic(Xcoords=CamX[flyremainvect], Ycoords=CamY[flyremainvect], IndVect=flyremainvect)
    #robot.notifyUserFail(flyremainvect, attImg=1)
#except:
    #print 'No failed arenas detected'

