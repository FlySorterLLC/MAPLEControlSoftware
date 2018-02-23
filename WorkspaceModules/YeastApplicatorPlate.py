##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

## Yeast applicator module workspace file.

import numpy as np

class YeastApplicatorPlate:
    def __init__(self, anchorX, anchorY):
        nrows = 7       # maximum possible rows and columns in arena
        ncols = 11
        rowstart = anchorY      # Y coord of first point of interest (POI)
        colstart = anchorX     # X coord of first POI
        xdif = 7.5       # x difference between POIs
        ydif = 7.5       # y difference between POIs
        self.POIz = 49
        self.Vacz = 50      
        arncoordsX = np.zeros((nrows, ncols))
        arncoordsY = np.zeros((nrows, ncols))
        curcoord = range(2)
        for arnrow in range(0,nrows):
            for arncol in range(0,ncols):
                curcoord[0] = colstart + (arncol * xdif)
                curcoord[1] = float(rowstart) - float(arnrow * ydif)
                arncoordsX[arnrow, arncol] = curcoord[0]
                arncoordsY[arnrow, arncol] = float(curcoord[1])
        self.ManipX = np.reshape(arncoordsX, (nrows*ncols,1))
        self.ManipY = np.reshape(arncoordsY, (nrows*ncols,1))
        if not any(self.ManipX <= 0) and not any(self.ManipY <= 0):
            print 'Yeast applicator plate successfully initialized.'
        else:
            print 'Check yeast applicator plate anchor coordinates. One or more coordinates out of bounds.'
        return

    def getApplicatorCoords(self, i):
        if ( i < 0 or i > 101 ):
            print "yeast applicator plate error: index out of bounds (", i, ")."
            return
        coords = np.array(range(2))
        coords[0] = self.ManipX[i]
        coords[1] = self.ManipY[i]
        return coords