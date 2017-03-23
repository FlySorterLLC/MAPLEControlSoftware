import cv2
import numpy as np
import time
import robotutil
import Workspace1
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser

robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.home()

MAX_SIZE=72
MIN_SIZE=58
startp1=119
startp2=150
startp3=2.7


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
POIz = 51       # depth of the POI openings
arncoordsX = np.zeros((nrows, ncols))
arncoordsY = np.zeros((nrows, ncols))
curcoord = range(2)
for arnrow in range(0,nrows):
    for arncol in range(0,ncols):
        curcoord[0] = colstart - (arncol * xdif)
        curcoord[1] = float(rowstart) - float(arnrow * ydif)
        arncoordsX[arnrow, arncol] = float(curcoord[0])
        arncoordsY[arnrow, arncol] = float(curcoord[1])
hcoordsX = np.zeros((nhrows, nhcols))
hcoordsY = np.zeros((nhrows, nhcols))
for hrow in range(0,nhrows):
    for hcol in range(0,nhcols):
        curcoord[0] = float(hcolstart) - float(hcol * hxdif)
        curcoord[1] = float(hrowstart) - float(hrow * hydif)
        hcoordsX[hrow, hcol] = float(curcoord[0])
        hcoordsY[hrow, hcol] = float(curcoord[1])
ManipX = np.reshape(arncoordsX, (nrows*ncols,1))
ManipY = np.reshape(arncoordsY, (nrows*ncols,1))
CamX = ManipX - camdiffx
CamY = ManipY - camdiffy
FlyHomeX = np.reshape(hcoordsX, (nhrows*nhcols,1))
FlyHomeY = np.reshape(hcoordsY, (nhrows*nhcols,1))
Radii = (ManipX / ManipX) * POIrad
print ManipX
print ManipY
print CamX
print CamY
print Radii


detectvect = robot.sweep(CamX[0:10], CamY[0:10])
detectx = ManipX[detectvect]
print detectx
detecty = ManipY[detectvect]
print detecty

numcircles = range(0,len(CamX))

for i,x in enumerate(CamX):
	robot.moveToSpd(pt=[float(CamX[i]), float(CamY[i]), 0, 40, 0, 5000])
	robot.light(True)
	time.sleep(0.2)
	robot.cam.start_live()
	robot.cam.snap_image()
	robot.cam.save_image(''.join(['curImage1.jpg']), 1, jpeq_quality=100)
	robot.cam.stop_live()
	robot.dwell(800)
	robot.light(False)
	image1 = cv2.imread('curImage1.jpg')
	image1 = cv2.resize(image1, (1280, 960))
	output = image1.copy()
	gray = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
	thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	cv2.imshow('image', thresh)
	key = cv2.waitKey(5000)
	if key == 27:
		break
	circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
	if  circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for j in range(0,len(circles)): 
			cv2.circle(output, (circles[j,0], circles[j,1]), circles[j,2], (0, 255, 0), 4)
			cv2.rectangle(output, (circles[j,0] - 5, circles[j,1] - 5), (circles[j,0] + 5, circles[j,1] + 5), (0, 128, 255), -1)
		numcircles[i] = len(circles)
		print len(circles)
	else:
		numcircles[i] = 0
	cv2.rectangle(output, (1280/2 - 5, 960/2 - 5), (1280/2 + 5, 960/2 + 5), (250, 250, 25), -1)
	cv2.imshow('image', output)
	key = cv2.waitKey(5000)
	if key == 27:
		break
print 'Circles found:', numcircles