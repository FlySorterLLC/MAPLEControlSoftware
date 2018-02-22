##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: SocialArena.py
#  Description: Contains and functions used to access social arena array.


## Dependencies
import numpy as np
import cv2

class YeastArena3x3:
    def __init__(self, anchorX, anchorY):
        nrows = 3       # maximum possible rows and columns in arena
        ncols = 3
        rowstart = anchorY      # Y coord of first point of interest (POI)
        colstart = anchorX     # X coord of first POI
        camdiffx = 44       # difference in cam and manip coords
        spdiffx = 81        # x difference with small part manipulator
        self.SPz = 50.4
        camdiffy = 6.3
        self.camsharpz = 20      # position at which POIs are sharp
        xdif = 99       # x difference between POIs
        ydif = 100       # y difference between POIs
        self.POIz = 49
        self.Vacz = 50
        self.agarZ = 39.5     # height at which to probe and place colonies
        arncoordsX = np.zeros((nrows, ncols))
        arncoordsY = np.zeros((nrows, ncols))
        curcoord = range(2)
        for arnrow in range(0,nrows):
            for arncol in range(0,ncols):
                curcoord[0] = colstart + (arncol * xdif)
                curcoord[1] = float(rowstart) + float(arnrow * ydif)
                arncoordsX[arnrow, arncol] = curcoord[0]
                arncoordsY[arnrow, arncol] = float(curcoord[1])
        self.ManipX = np.reshape(arncoordsX, (nrows*ncols,1))
        self.ManipY = np.reshape(arncoordsY, (nrows*ncols,1))
        self.CamX = self.ManipX - camdiffx
        self.CamY = self.ManipY - camdiffy
        self.SmallPX = self.ManipX - spdiffx
        self.SmallPY = self.ManipY      
        if not any(self.ManipX <= 0) and not any(self.ManipY <= 0) and not any(self.CamX <= 0) and not any(self.CamY <= 0):
            print 'Yeast arena array successfully initialized.'
        else:
            print 'Check yeast arena anchor coordinates. One or more coordinates out of bounds.'

        return

    def getArenaCoords(self, i):
        if ( i < 0 or i > 81 ):
            print "Yeast arena error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.ManipX[i]
        coords[1] = self.ManipY[i]
        return coords

    def getCamCoords(self, i):
        if ( i < 0 or i > 81 ):
            print "Yeast arena error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.CamX[i]
        coords[1] = self.CamY[i]
        return coords

    def getSPCoords(self, i):
        if ( i < 0 or i > 81 ):
            print "Yeast arena error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.SmallPX[i]
        coords[1] = self.SmallPY[i]
        return coords