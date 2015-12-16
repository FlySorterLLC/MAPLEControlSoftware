#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Triangular maze array (original style) class file

import numpy as np

class TriangleMaze:

    Z2ClearanceHeight = 22.
    Z2Height = 30.
    horizontalMazeSpacing = 36.274 #mm
    verticalMazeSpacing = 36.466 #mm
    verticalShift = 14.47 #mm
    mazeRows = 8
    mazeCols = 8

    # maze1 is the coordinates of the center of maze number 1 (upper left, sorta)
    # maze64 is the coordinates of the center of maze number 64 (bottom right)
    def __init__(self, maze1, maze64):

        self.startMazePoint = maze1

        # Build basis vectors as if the maze is aligned with the X axis
        self.majorBasis = np.array([self.horizontalMazeSpacing, 0.])
        self.minorBasis = np.array([0., -self.verticalMazeSpacing])
        self.invertedOffset = np.array([self.horizontalMazeSpacing/2., self.verticalShift])

        # Check the distance between the two maze coords for sanity
        # Should 
        l = np.linalg.norm(maze64-maze1)
        nominalVect = (self.mazeCols-1)*self.majorBasis + (self.mazeRows-1)*self.minorBasis
        nominalDist = np.linalg.norm( nominalVect )
        if ( np.absolute( 1 - l / ( nominalDist )) > 0.01 ):
            print "Maze warning: check coordinates. Distance is", l, "but should be", nominalDist


        # Now calculate the angle (well, sin and cos of the angle) between the actual
        # coordinates and the axis-aligned coordinates
        cosAngle =  np.dot(   (maze64-maze1), nominalVect ) / (l * nominalDist)
        sinAngle = -np.cross( (maze64-maze1), nominalVect ) / (l * nominalDist)
        #print "cos(t) =", cosAngle, "and sin(t) =", sinAngle

        # Construct the rotation matrix
        rotMat = np.array( [ [cosAngle, -sinAngle], [ sinAngle, cosAngle] ])

        # Now apply the rotation matrix to the basis vectors
        self.majorBasis = rotMat.dot(self.majorBasis)
        self.minorBasis = rotMat.dot(self.minorBasis)
        self.invertedOffset = rotMat.dot(self.invertedOffset)
        
        return

    # Note that this index (i) is one based and matches the numbering on the mazes.
    def getMaze(self, i):
        if ( i < 1 or i > 120 ):
            print "Maze error: index out of bounds (", i, ")."
            return

        # Math is a little different for 1-64 and 65-120
        if ( i <= (self.mazeRows*self.mazeCols) ):
            coords = self.startMazePoint + int((i-1)/self.mazeCols)*self.minorBasis + ((i-1)%self.mazeCols)*self.majorBasis
        else:
            # Adjust the counter, so 65 is now 1 (makes math easier)
            i -= (self.mazeRows*self.mazeCols)
            coords = self.startMazePoint + self.invertedOffset + int( (i-1)/(self.mazeCols-1) ) * self.minorBasis + ( (i-1)%(self.mazeCols-1))*self.majorBasis

        return coords