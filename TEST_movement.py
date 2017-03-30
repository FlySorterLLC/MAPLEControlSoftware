#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Hardcoded program to pick & deposit flies

import cv2
import numpy as np
import time
import robotutil
import Workspace1
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser

#### BEGIN PGM ####
coordfromcfg = False       # Set True to read Arena_Coordinates.cfg file for coordinates
robot = robotutil.santaFe("SantaFe.cfg")


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
    POIz = 51
    Vacz = 51       # depth from which to vacuum the fly out of the POI
    homezdepos = 43      # depth at which flies can be deposited in home
    homezwd = 45        # depth at which flies can be vacuumed out of home


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
# Construct some helpful points
robot.smoothie.sendCmd("M999")
robot.home()
homepos = robot.getCurrentPosition()
misses = 0
iterations = 10     # denote amounts of rows as arenas * 9
startarena = 0
repeats = 1
fails = np.zeros(iterations * repeats)
recheck = np.zeros(iterations * repeats)
x = np.arange(iterations * repeats)
y = np.ones(iterations * repeats)

robot.smallPartManipVac(False)

for j in range(0,repeats):
    for i in range(startarena,iterations):
        flyremaining = True
        robot.moveToSpd(pt=[float(CamX[i]), float(CamY[i]), 0, camsharpz, 10, 5000])
        robot.dwell(t=1)
        degs1 = int(robot.findDegs(slowmode=True))
        robot.dwell(5)
        robot.moveToSpd(pt=[float(ManipX[i]), float(ManipY[i]), 0, camsharpz, 10, 5000])    
        robot.dwell(t=10)
        Mid1 = robot.getCurrentPosition()
        robot.dwell(1)
        starttime = time.time()
        while flyremaining == True:
            if degs1 <= 180:        # hastens bringing the opening to a vacuumable position
                endpos1 = 50
            elif degs1 > 180:
                endpos1 = 130
            tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = Radii[i], z = POIz, startpos=degs1, endpos=endpos1, spd=2000)
            #print 'tempcoords at enddeg', tempCoord['endDeg']
            misses = misses + tempCoord['limit']     # vac 1 sec at endpoint
            robot.dwell(50)
            #robot.home()
            #robot.moveXY(pt=tempCoord['oldMid'])
            #robot.dwell(50)
            #robot.moveTo(pt=[tempCoord['oldMid'][0],tempCoord['oldMid'][1], 0, 0, 40])
            #print 'moving down a bit'
            #robot.dwell(t=100)
            #tempCircEnd = robot.moveCirc2(mid = tempCoord['oldMid'], r = 10.8, z = 53, startpos=tempCoord['endDeg'], endpos=rand.choice(range(360)), spd=1000)
            #robot.dwell(100)
            #robot.moveZ(pt=[Mid1[0],Mid1[1],0,0,0])
            fails[i+(j*iterations)] = tempCoord['limit']
            if (i+j) > 1 and (i*j) < iterations*repeats:        # skip evaluating first and last
                if fails[i+(j*iterations)] and fails[i+(j*iterations)-1]:       # 2 fails in a row should reset coordinates
                    print 'possible misalignment - resetting...'
                    fails[i+(j*iterations)] = 0     # so it remains the same chance after reset
                    robot.home()
                    break
            if tempCoord['limit'] != 1:     
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,Vacz])
                robot.smallPartManipVac(False)
                for b in range(0,2):
                    robot.smallPartManipVac(True)
                    robot.dwell(t=200)
                    robot.smallPartManipVac(False)
                    robot.dwell(t=30)
                robot.smallPartManipVac(True)
                robot.dwell(t=400)
                robot.smallPartManipVac(False)
                robot.dwell(t=5)
                robot.smallPartManipVac(True)
                if tempCoord['endDeg'] <= 180:        # hastens bringing the opening to a vacuumable position
                    endpos2 = 1
                elif tempCoord['endDeg']  > 180:
                    endpos2 = 180
                tempCoord = robot.tryOpening(mid = tempCoord['oldMid'], r = float(Radii[i]), z = POIz, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=1)
                endpos1 = tempCoord['endDeg']
                robot.dwell(50)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
                robot.dwell(50)
            # Check if fly is vacuumed out
            robot.moveToSpd(pt=[float(CamX[i]), float(CamY[i]), 0, camsharpz, 10, 5000])
            robot.dwell(50)
            flyremaining = robot.detectFly( minpx=40, maxpx=2000)
            if flyremaining == 1:
                degs1 = int(robot.findDegs(slowmode=True))
                recheck[i] = 1


        robot.moveToSpd(pt=[float(FlyHomeX[0]), float(FlyHomeY[0]), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
        robot.dwell(t=1)
        robot.moveToSpd(pt=[float(FlyHomeX[i]), float(FlyHomeY[i]), 0, 0, 10, 5000])        # Go to actual home at index i
        robot.dwell(t=1)
        trylowerHome = robot.lowerCare(z=homezdepos, descendZ=7, retreatZ=7)      # Move into home - check Z height!
        robot.dwell(t=50)
        if trylowerHome['limit'] == 0:
            robot.smallPartManipVac(False)

            robot.dwell(t=1)
            robot.flyManipAir(True)
            robot.dwell(t=6)
            robot.flyManipAir(False)
            robot.dwell(t=3)
            robot.moveRel(pt=[0, 0, 0, 0, -homezdepos])
            robot.dwell(t=1)
        else:
            print 'possible misalignment - resetting...'
            fails[i+(j*iterations)] = 0     # so it remains the same chance after reset
            robot.home()
            break

        print 'so far', misses, 'complete misses out of', (i+1)+(j*iterations), 'trials.'
        endtime = time.time()
        y[(i+(j*iterations)-1)] = endtime - starttime
        print 'trial', (i+1) + (j*iterations), 'took', y[(i+(j*iterations))-1], 'seconds to complete'
        starttime = 0
        endtime = 0
    robot.home()


#robot.rotate(pt=[148, 160, 0, 0, 0, 0, -11,])                   
#robot.moveXY(pt=[0,0])
totalmisses = float(misses)/float(repeats*iterations)
print 'missed the hole completely', (totalmisses*100), 'percent of the time.'
print 'processing', repeats*iterations, 'arenas took', sum(y), 'seconds, with a mean time of', np.mean(y), 'and std of', np.std(y)

# Sweep over arenas that took long to process
print 'rechecking arenas that were slower than', (np.mean(y) + np.std(y)*1.4),'seconds to process'
for s in range(startarena, iterations):
    if y[s] > (np.mean(y) + np.std(y)*1.4) or recheck[s] == 1:
        print 'sweeping arena', s, '...'
        robot.moveToSpd(pt=[float(CamX[s]), float(CamY[s]), 0, camsharpz, 10, 5000])
        flyremaining = robot.detectFly(minpx=40, maxpx=2000)
        if flyremaining == True:
            robot.dwell(t=1)
            degs1 = int(robot.findDegs(slowmode=True))
            robot.dwell(5)
            robot.moveToSpd(pt=[float(ManipX[s]), float(ManipY[s]), 0, camsharpz, 10, 5000])    
            robot.dwell(t=1)
            Mid1 = robot.getCurrentPosition()
            robot.dwell(1)
            while flyremaining == True:
                if degs1 <= 180:        # hastens bringing the opening to a vacuumable position
                    endpos1 = 50
                elif degs1 > 180:
                    endpos1 = 130
                tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(Radii[s]), z = POIz, startpos=degs1, endpos=endpos1, spd=2000)
                robot.dwell(5)
                fails = tempCoord['limit']
                if 1 < s < iterations:        # skip evaluating first and last
                    if fails == 1:       # 2 fails in a row should reset coordinates
                        print 'possible misalignment - check arena ', s, '!'
                        fails = 0     # so it remains the same chance after reset
                        robot.home()
                        break

                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,Vacz])
                robot.smallPartManipVac(False)
                for b in range(0,3):
                    robot.smallPartManipVac(True)
                    robot.dwell(t=rand.choice(range(250,350)))
                    robot.smallPartManipVac(False)
                    robot.dwell(t=rand.choice(range(40,90)))
                robot.smallPartManipVac(True)
                if tempCoord['endDeg'] <= 180:        # hastens bringing the opening to a vacuumable position
                    endpos2 = 1
                elif tempCoord['endDeg']  > 180:
                    endpos2 = 180
                tempCoord = robot.tryOpening(mid = tempCoord['oldMid'], r = float(Radii[s]), z = POIz, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000)
                robot.dwell(50)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,30])
                # Check if fly is vacuumed out
                robot.moveToSpd(pt=[float(CamX[s]), float(CamY[s]), 0, camsharpz, 10, 5000])
                flyremaining = robot.detectFly(minpx=40, maxpx=2000)
                if flyremaining == 1:
                    degs1 = int(robot.findDegs(slowmode=True))


            robot.moveToSpd(pt=[float(FlyHomeX[0]), float(FlyHomeY[0]), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
            robot.dwell(t=1)
            robot.moveToSpd(pt=[float(FlyHomeX[s]), float(FlyHomeY[s]), 0, 0, 10, 5000])        # Go to actual home at index i
            robot.dwell(t=1)
            trylowerHome = robot.lowerCare(z=homezdepos, descendZ=7, retreatZ=7)      # Move into home - check Z height!
            robot.dwell(t=50)
            if trylowerHome['limit'] == 0:
                robot.smallPartManipVac(False)
                robot.dwell(t=1)
                robot.flyManipAir(True)
                robot.dwell(t=6)
                robot.flyManipAir(False)
                robot.dwell(t=3)
                robot.moveRel(pt=[0, 0, 0, 0, -homezdepos])
                robot.dwell(t=1)
robot.home()

                                                        ##### UNLOADING STEP DONE #####


recheck = np.zeros(iterations * repeats)
consectryunload = 0
fails = np.zeros(iterations * repeats)

for j in range(0,repeats):      # LOADING FLIES INTO ARENA
    for i in range(startarena,iterations):
        starttime = time.time()
        flyremaining = False
        while flyremaining == False:
            robot.moveToSpd(pt=[float(FlyHomeX[0]), float(FlyHomeY[0]), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
            robot.dwell(t=1)
            robot.moveToSpd(pt=[float(FlyHomeX[i]), float(FlyHomeY[i]), 0, 0, 10, 5000])        # Go to actual home at index i
            robot.dwell(t=1)
            trylowerHome = robot.lowerCare(z=homezwd, descendZ=9, retreatZ=9)      # Move into home - check Z height!
            if trylowerHome['limit'] == 0:
                robot.dwell(t=1)
                for b in range(0,1):
                    robot.flyManipAir(True)
                    robot.dwell(t=rand.choice(range(2,4)))
                    robot.flyManipAir(False)
                    robot.dwell(t=rand.choice(range(2,4)))
                robot.smallPartManipVac(True)
                robot.dwell(t=2000)
                robot.moveRel(pt=[0, 0, 0, 0, -homezwd])
                robot.dwell(t=1)

            robot.moveToSpd(pt=[float(CamX[i]), float(CamY[i]), 0, camsharpz, 10, 5000])
            robot.dwell(t=1)
            degs1 = int(robot.findDegs(slowmode=True))
            robot.dwell(5)
            robot.moveToSpd(pt=[float(ManipX[i]), float(ManipY[i]), 0, camsharpz, 10, 5000])    
            robot.dwell(t=1)
            Mid1 = robot.getCurrentPosition()
            robot.dwell(1)
            if degs1 <= 180:        # hastens bringing the opening to a vacuumable position
                endpos1 = 50
            elif degs1 > 180:
                endpos1 = 130
            tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(Radii[i]), z = POIz, startpos=degs1, endpos=endpos1, spd=2000)
            #print 'tempcoords at enddeg', tempCoord['endDeg']
            misses = misses + tempCoord['limit']     # vac 1 sec at endpoint
            robot.dwell(5)
            fails[i+(j*iterations)] = tempCoord['limit']
            if (i+j) > 1 and (i*j) < iterations*repeats:        # skip evaluating first and last
                if fails[i+(j*iterations)] and fails[i+(j*iterations)-1]:       # 2 fails in a row should reset coordinates
                    print 'possible misalignment - resetting...'
                    fails[i+(j*iterations)] = 0     # so it remains the same chance after reset
                    robot.home()
                    break
            if tempCoord['limit'] != 1:     
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,Vacz])
                for b in range(0,1):
                    robot.smallPartManipVac(False)
                    robot.dwell(5)
                    robot.flyManipAir(True)
                    robot.dwell(t=5)
                    robot.flyManipAir(False)
                    robot.dwell(t=2)
                if tempCoord['endDeg'] <= 180:        # hastens bringing the opening to a vacuumable position
                    endpos2 = 1
                elif tempCoord['endDeg']  > 180:
                    endpos2 = 360
                tempCoord = robot.tryOpening(mid = tempCoord['oldMid'], r = float(Radii[i]), z = POIz, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=1)
                endpos1 = tempCoord['endDeg']
                robot.dwell(50)
                robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
            # Check if fly is vacuumed out
            robot.moveToSpd(pt=[float(CamX[i]), float(CamY[i]), 0, camsharpz, 10, 5000])
                            
                            ### COMMENT BACK IN IN THE MORNING! ###
            flyremaining = robot.detectFly( minpx=40, maxpx=2000)
            if flyremaining == False and consectryunload >= 2:
                flyremaining = True        # Move on after 3 tries and mark for sweep
                consectryunload = 0
                recheck[i] = 1
                print 'could not load arena', i
            elif flyremaining == False:
                consectryunload = consectryunload + 1
                print 'Unloaded fly', i, 'not found moving, retrying...'
            elif flyremaining == True:
                consectryunload = 0


        print 'so far', misses, 'complete misses out of', (i+1)+(j*iterations), 'trials.'
        endtime = time.time()
        y[(i+(j*iterations)-1)] = endtime - starttime
        print 'trial', (i+1) + (j*iterations), 'took', y[(i+(j*iterations))-1], 'seconds to complete'
        starttime = 0
        endtime = 0
    robot.home()


#robot.rotate(pt=[148, 160, 0, 0, 0, 0, -11,])                   
#robot.moveXY(pt=[0,0])
totalmisses = float(misses)/float(repeats*iterations)
print 'missed the hole completely', (totalmisses*100), 'percent of the time.'
print 'processing', repeats*iterations, 'arenas took', sum(y), 'seconds, with a mean time of', np.mean(y), 'and std of', np.std(y)