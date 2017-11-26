## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: SocialArena.py
#  Description: Contains and functions used to access social arena array.


## Dependencies
import numpy as np
import cv2

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
