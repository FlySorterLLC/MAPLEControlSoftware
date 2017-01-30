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
robot = robotutil.santaFe("SantaFe.cfg")

print "Reading arena camera and manipulator coordinates...",
config = ConfigParser.RawConfigParser()
config.read("Arena_Coordinates.cfg")
CamX = config.get('COORDINATES', 'XCam')
CamY = config.get('COORDINATES', 'YCam')
ManipX = config.get('COORDINATES', 'XManip')
ManipY = config.get('COORDINATES', 'YManip')
Radii = config.get('COORDINATES', 'Radii')

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

print "done."

# Construct some helpful points
robot.smoothie.sendCmd("M999")
robot.home()
homepos = robot.getCurrentPosition()
misses = 0
iterations = 18
startarena = 0
repeats = 10
x = np.arange(iterations * repeats)
y = np.arange(iterations * repeats)
for j in range(0,repeats):
    for i in range(startarena,iterations):
        robot.moveToSpd(pt=[float(CamX[i+1]), float(CamY[i+1]), 0, 45, 10, 5000])
        robot.dwell(t=1)
        degs1 = int(robot.findDegs(slowmode=True))
        robot.dwell(5)
        robot.moveToSpd(pt=[float(ManipX[i+1]), float(ManipY[i+1]), 0, 45, 10, 5000])       # +45.5x, +7.5y from camera to manipulator
        robot.dwell(t=1)
        Mid1 = robot.getCurrentPosition()
        robot.dwell(1)
        randpos = rand.choice(range(360))
        starttime = time.time()
        tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(Radii[i+1]), z = 53, startpos=degs1, endpos=randpos, spd=1000)
        #print 'tempcoords at enddeg', tempCoord['endDeg']
        misses = misses + tempCoord['limit']
        robot.dwell(1)     #to make sure it stopped circling
        robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
        #robot.home()
        #robot.moveXY(pt=tempCoord['oldMid'])
        #robot.dwell(50)
        #robot.moveTo(pt=[tempCoord['oldMid'][0],tempCoord['oldMid'][1], 0, 0, 40])
        #print 'moving down a bit'
        #robot.dwell(t=100)
        #tempCircEnd = robot.moveCirc2(mid = tempCoord['oldMid'], r = 10.8, z = 53, startpos=tempCoord['endDeg'], endpos=rand.choice(range(360)), spd=1000)
        #robot.dwell(100)
        #robot.moveZ(pt=[Mid1[0],Mid1[1],0,0,0])
        print 'so far', misses, 'complete misses out of', (i+1)+(j*10), 'trials.'
        endtime = time.time()
        y[(i+(j*10)-1)] = endtime - starttime
        print 'trial', (i+1) + (j*10), 'took', y[(i+(j*10))-1], 'seconds to complete'
        starttime = 0
        endtime = 0


#robot.rotate(pt=[148, 160, 0, 0, 0, 0, -11,])                   
#robot.moveXY(pt=[0,0])
robot.release()
totalmisses = float(misses)/float(repeats*iterations)
print 'missed the hole completely', (totalmisses*100), 'percent of the time.'

plt.plot(x, y)
plt.show()
print 'processing', repeats*iterations, 'arenas took', sum(y), 'seconds'