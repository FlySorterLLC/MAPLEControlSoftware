## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: SocialArena.py
#  Description: Contains and functions used to access social arena array.


## Dependencies
import os
import math
import cv2
import numpy as np
import sys
import traceback
import glob
import serial
import time
import ConfigParser
import random as rand
import pyicic.IC_ImagingControl
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil as robot

class SocialArena:
    def __init__(self, anchorX, anchorY):
        nrows = 9       # maximum possible rows and columns in arena
        ncols = 9
        rowstart = anchorX      # Y coord of first point of interest (POI)
        colstart = anchorY     # X coord of first POI
        camdiffx = 44       # difference in cam and manip coords
        camdiffy = 6.3 
        camsharpz = 40      # position at which POIs are sharp
        xdif = 32.1       # x difference between POIs
        ydif = 32       # y difference between POIs
        POIrad = 10.5     # radius of arena POI opening
        POIz = 49
        Vacz = 50       # depth from which to vacuum the fly out of the POI
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
        self.ManipX = np.reshape(arncoordsX, (nrows*ncols,1))
        self.ManipY = np.reshape(arncoordsY, (nrows*ncols,1))
        self.CamX = self.ManipX - camdiffx
        self.CamY = self.ManipY - camdiffy
        self.Radii = (self.ManipX / self.ManipX) * POIrad
        if not any(self.ManipX <= 0) and not any(self.ManipY <= 0) and not any(self.CamX <= 0) and not any(self.CamY <= 0):
            print 'Social arena array successfully initialized.'
        else:
            print 'Check anchor coordinates. One or more coordinates out of bounds.'

        return  

    def getArenaCoords(self, i):
        if ( i < 0 or i > 81 ):
            print "Social arena array error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.ManipX[i]
        coords[1] = self.ManipY[i]
        return coords

    def getCamCoords(self, i):
        if ( i < 0 or i > 81 ):
            print "Social arena array error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.CamX[i]
        coords[1] = self.CamY[i]
        return coords

    # Uses repeated hit-detection of moveCirc2() as error-correction of slightly deviating opening-detection
    def tryOpening(self, mid, r, n=360, startpos=0, endpos=360, spd=1000, rest=5, z=53, full=True, retreatZ=10, descendZ=9):
        tryspd = spd
        trymid = mid
        trystart = startpos
        tryend = endpos
        tryz = z
        unsure=0
        radi = r
        careonce = 0
        trylower = robot.moveCirc2(mid=trymid, r=radi, n=360, startpos=trystart, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
        startposFirst = startpos
        while trylower['limit'] == 1 and unsure != 1:
            for cw in xrange(2,10,2):
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos + cw
                    trylower = robot.moveCirc2(mid=trymid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            startpos = startposFirst
            for ccw in xrange(2,10,2):       # can skip 0 degree offset due to clockwise motion
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos - ccw
                    trylower = robot.moveCirc2(mid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            if trylower['limit'] == 1:
                print 'Could not find opening - detecting anew...'
                unsure = 1
                robot.homeZ()
        trylower.update({'limitonce':careonce})
        return trylower

    # Highest order command to withdraw a single fly from a single arena in the behavioral module (Different withdraw-strategies accessible using vacstrategy)
    def arenaWithdraw(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, vacPos, vacZ, closePos, vacstrategy=2, vacBurst=1, imgshow=0):
        strategy = vacstrategy
        missonce = 0
        print 'Using strategy', strategy
        robot.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
        robot.dwell(t=1)
        degs1 = int(robot.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
        robot.dwell(t=5)
        robot.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
        robot.dwell(t=10)
        Mid1 = robot.getCurrentPosition()
        robot.dwell(1)
        endpos1 = vacPos
        if strategy == 3 or strategy == 6:
            robot.smallPartManipVac(True)
        tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
        miss = tempCoord['limit']
        missonce = missonce + tempCoord['limitonce']
        robot.dwell(50)
        if miss == 1:
            print 'Possible misalignment - resetting...'
            robot.home()
            return {'miss': miss, 'missonce': missonce}
        elif miss != 1:
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            if strategy == 1:
                robot.smallPartManipVac(False)
                for b in range(0,vacBurst):
                    robot.smallPartManipVac(True)
                    robot.dwell(t=200)
                    robot.smallPartManipVac(False)
                    robot.dwell(t=10)
                robot.smallPartManipVac(True)
                robot.dwell(t=400)
                robot.smallPartManipVac(False)
                robot.dwell(t=5)
                robot.smallPartManipVac(True)
            elif strategy == 2:
                robot.smallPartManipVac(False)
                robot.flyManipAir(True)
                robot.dwell(t=5)
                robot.flyManipAir(False)
                robot.dwell(t=5)
                robot.flyManipAir(True)
                robot.dwell(t=5)
                robot.flyManipAir(False)
                robot.dwell(t=5)
                robot.smallPartManipVac(True)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ+1])
                robot.dwell(t=50)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ-1])
                robot.dwell(t=50)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                robot.dwell(t=50)
            elif strategy == 3:
                sweeppos1 = tempCoord['endDeg']
                if sweeppos1 < 180:
                    sweeppos2 = 140
                else:
                    sweeppos2 = 220
                checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
                missonce = missonce + checkformiss['limitonce']
                robot.dwell(50)
                checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
                missonce = missonce + checkformiss['limitonce']
            elif strategy == 4:
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                robot.dwell(t=10)
                robot.smallPartManipVac(True)
                robot.dwell(t=20000)
            elif strategy == 5:
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                robot.dwell(t=5)
                robot.flyManipAir(True)
                robot.dwell(t=500)
                robot.flyManipAir(False)
                robot.smallPartManipVac(True)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                robot.dwell(t=1000)
            elif strategy == 6:
                sweeppos1 = tempCoord['endDeg']
                if sweeppos1 < 180:
                    sweeppos2 = 140
                else:
                    sweeppos2 = 220
                robot.flyManipAir(True)
                robot.dwell(t=5)
                robot.flyManipAir(False)
                self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
                robot.flyManipAir(True)
                robot.dwell(t=5)
                robot.flyManipAir(False)
                self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
                robot.dwell(50)
            endpos2 = closePos
            tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
            missonce = missonce + tempCoord['limitonce']
            endpos1 = tempCoord['endDeg']
            robot.dwell(10)
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
            robot.dwell(10)
            return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

    # Highest order command to deposit a single fly in a single arena in the behavioral module
    def arenaDeposit(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, airPos, airZ, closePos, airBurst=1, imgshow=0):
        missonce = 0
        robot.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
        robot.dwell(t=1)
        degs1 = int(robot.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
        robot.dwell(t=5)
        robot.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
        robot.dwell(t=10)
        Mid1 = robot.getCurrentPosition()
        robot.dwell(1)
        endpos1 = airPos
        tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
        miss = tempCoord['limit']
        missonce = missonce + tempCoord['limitonce']
        robot.dwell(50)
        if miss == 1:
            print 'Possible misalignment - full reset...'
            robot.home()
            return {'miss': miss, 'missonce': missonce}
        elif miss != 1:
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,airZ])
            robot.smallPartManipVac(False)
            robot.dwell(t=5)
            for b in range(0,airBurst):
                robot.flyManipAir(True)
                robot.dwell(t=rand.choice(range(5,6)))
                robot.flyManipAir(False)
                robot.dwell(t=rand.choice(range(5,6)))
            robot.dwell(t=50)
            endpos2 = closePos
            tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
            missonce = missonce + tempCoord['limitonce']
            endpos1 = tempCoord['endDeg']
            robot.dwell(50)
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
            robot.dwell(10)
            return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}
