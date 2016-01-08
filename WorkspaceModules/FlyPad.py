#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly pad class file

import numpy as np
import cv2

class FlyPad:

    MaxThickness = 15.3
    Z0WorkingThickness = 15.3
    Z1WorkingThickness = 15.3
    Z2WorkingThickness = 15.3

    # upperLeft is coordinate of upper left corner,
    # lowerRight is coord of lower right.
    # This code assumes the pad is aligned with the
    # major axis of 
    def __init__(self, upperLeft, lowerRight):
        if ( ( upperLeft[0] > lowerRight[0] ) or
             ( upperLeft[1] < lowerRight[1] ) ):
            print "Fly pad error: coordinates not properly oriented."
            return
        
        self.upperLeft = upperLeft
        self.lowerRight = lowerRight

        self.width = self.lowerRight[0]-self.upperLeft[0]
        self.height = self.upperLeft[1]-self.lowerRight[1]
                
        return

    def calculateRegionCoords(self, FOV):
        # robot.FOV contains the horizontal and vertical fields of view
        # So we want to break up the pad into points that
        # are the centers of regions that overlap by a small amount

        overlap = 2 #mm
        self.numRegions = ( int( self.width / (FOV[0]-overlap) )+1,
                            int( self.height / (FOV[1]-overlap) )+1 )
        firstXCoord = self.upperLeft[0] + (FOV[0]/2.) #- (overlap/2.)
        lastXCoord = self.lowerRight[0] - (FOV[0]/2.) #+ (overlap/2.)
        firstYCoord = self.lowerRight[1] + (FOV[1]/2.) #- (overlap/2.)
        lastYCoord = self.upperLeft[1] - (FOV[1]/2.) #+ (overlap/2.)
        XSpacing = (lastXCoord-firstXCoord)/float(self.numRegions[0]-1)
        YSpacing = (lastYCoord-firstYCoord)/float(self.numRegions[1]-1)
        self.imagePoints = np.zeros((self.numRegions[0]*self.numRegions[1], 2))
        i = 0
        for x in range(0, self.numRegions[0]):
            for y in range(0, self.numRegions[1]):
                self.imagePoints[i] = [firstXCoord + x * XSpacing,
                                       firstYCoord + y * YSpacing ]
                i+=1
        return

    def findNextFly(self, robot):
        pass
    
