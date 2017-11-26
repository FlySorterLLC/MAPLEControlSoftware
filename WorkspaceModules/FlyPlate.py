#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## FlyPlate (96 well plate) class file

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

class FlyPlate:

    MaxThickness = 19.
    Z2WorkingThickness = 12.
    
    wellSpacing = 9.

    # startWellPoint is well A1 (upper left), endWellPoint is well H12 (lower right)
    def __init__(self, startWellPoint, endWellPoint):

        self.startWellPoint = startWellPoint

        # The basis vectors, if the long side of the plate
        # was aligned to the X axis
        self.majorBasis = np.array([9., 0.])
        self.minorBasis = np.array([0., -9.])

        # First, check the length of endWellPoint-startWellPoint
        # against the nominal distance for sanity
        l = np.linalg.norm(endWellPoint-startWellPoint)
        nominalVect = 11*self.majorBasis + 7*self.minorBasis
        nominalDist = np.linalg.norm( nominalVect)
        if ( np.absolute( 1 - l / (nominalDist) ) > 0.01 ):
            print "FlyPlate warning: check coordinates. Length should be", (9.*np.sqrt(170.)), "but is", l


        # Now calculate the angle between the actual coords and the axis-aligned coords
        # reminder: a * b = |a| |b| cos(t)
        #     and   a x b = |a| |b| sin(t)
        #
        # We solve for sin(t) and cos(t)
        cosAngle = np.dot(   (endWellPoint-startWellPoint), nominalVect ) / ( l * nominalDist )
        sinAngle = -np.cross( (endWellPoint-startWellPoint), nominalVect ) / ( l * nominalDist )
        #print "cos(t) =", cosAngle, "and sin(t) =", sinAngle

        # Construct the rotation matrix                               
        rotMat = np.array( [ [cosAngle, -sinAngle], [ sinAngle, cosAngle] ])

        # Now apply the rotation matrix to the basis vectors
        self.majorBasis = rotMat.dot(self.majorBasis)
        self.minorBasis = rotMat.dot(self.minorBasis)

        return

    # Note that this index (i) is zero based.
    # So getWell(0) returns coords for well A1
    # getWell(1) returns coords for well A2
    # ...
    # and getWell(95) returns coords for well H12
    def getWell(self, i):
        if ( i < 0 or i > 95 ):
            print "FlyPlate error: index out of bounds (", i, ")."
            return
        coords = self.startWellPoint + (int(i/12))*self.minorBasis + (i%12)*self.majorBasis
        return coords

    # Highest order command to withdraw a single fly from a single well in the housing module
    def homeWithdraw(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, dislodgeZ=10, vacBurst=1, vacDur=4000, homeZ=45):
        if refptX != 'N':
            robot.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
            robot.dwell(t=1)
        robot.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, dislodgeZ, 5000])        # Go to actual home
        robot.dwell(t=1)
        robot.flyManipAir(True)
        trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
        if trylowerHome['limit'] == 0:
            robot.dwell(t=1)
            for b in range(0,vacBurst):
                robot.flyManipAir(False)
                robot.smallPartManipVac(True)
                robot.dwell(t=rand.choice(range(2,4)))
                robot.smallPartManipVac(False)
                robot.dwell(t=rand.choice(range(2,4)))
            robot.smallPartManipVac(True)
            robot.dwell(t=vacDur)
            robot.moveRel(pt=[0, 0, 0, 0, -homeZ])
            robot.dwell(t=10)
        else:
            robot.flyManipAir(False)
            robot.home()
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

    # Highest order command to deposit a single fly in a well in the housing module
    def homeDeposit(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44):
        if refptX != 'N':
            robot.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
            robot.dwell(t=1)
        robot.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, 10, 5000])        # Go to actual home
        robot.dwell(t=1)
        trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
        if trylowerHome['limit'] == 0:
            robot.dwell(t=1)
            robot.smallPartManipVac(False)
            for b in range(0,vacBurst):
                robot.flyManipAir(True)
                robot.dwell(t=rand.choice(range(5,6)))
                robot.flyManipAir(False)
                robot.dwell(t=rand.choice(range(5,6)))
            robot.dwell(t=50)
            robot.moveRel(pt=[0, 0, 0, 0, -homeZ])
            robot.dwell(t=10)
        else:
            robot.home()
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}
