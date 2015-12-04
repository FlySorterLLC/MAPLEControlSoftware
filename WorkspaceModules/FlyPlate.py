#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## FlyPlate (96 well plate) class file

import numpy as np
import robotutil

class FlyPlate:

    ZClearanceHeight = 50
    ZHeight = 45
    wellSpacing = 9.

    # startWellPoint is well A1 (upper left), endWellPoint is well H12 (lower right)
    def __init__(self, startWellPoint, endWellPoint):

        self.startWellPoint = startWellPoint

        # First, check the length of endWellPoint-startWellPoint for sanity
        # Should be wellSpacing*sqrt(7*7 + 11*11) = 9*sqrt(170) = ~117.34
        l = np.linalg.norm(endWellPoint-startWellPoint)
        if ( np.absolute( 1 - l / (9.*np.sqrt(170.)) ) > 0.01 ):
            print "FlyPlate warning: check coordinates. Length should be", (9.*np.sqrt(170.)), "but is", l

        # The basis vectors, if the long side of the plate
        # was aligned to the X axis
        self.majorBasis = np.array([9., 0.])
        self.minorBasis = np.array([0., -9.])


        # Now calculate the angle between the actual coords and the axis-aligned coords
        cosAngle = np.dot( (endWellPoint-startWellPoint), (11.*self.majorBasis+7.*self.minorBasis) ) / l / (9.*np.sqrt(170.))
        sinAngle = np.cross( (endWellPoint-startWellPoint), (11.*self.majorBasis+7.*self.minorBasis) ) / l / (9.*np.sqrt(170.))

        # Construct the rotation matrix                               
        rotMat = np.array( [ [cosAngle, -sinAngle], [ sinAngle, cosAngle] ])

        # Now apply the rotation matrix to the basis vectors
        self.majorBasis = rotMat.dot(self.majorBasis)
        self.minorBasis = rotMat.dot(self.minorBasis)

        return

    # Note that this index (i) is zero based.
    # So getWell(0) returns coords for well A1
    # and getWell(95) returns coords for well H12
    def getWell(self, i):
        coords = self.startWellPoint + (int(i/8))*self.majorBasis + (i%8)*self.minorBasis
        return coords
