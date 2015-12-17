#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## FlyPlate (96 well plate) class file

import numpy as np

class FlyPlate:

    MaxThickness = 19.
    Z0WorkingThickness = 16.
    
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
