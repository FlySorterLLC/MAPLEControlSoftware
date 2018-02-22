##
#
#  File: Example_CElegansTracking.py
#  Description:

## Dependencies
import cv2
import numpy as np
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
from datetime import datetime

#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)

duration = 600		# in seconds (1 fps)
robot.moveToSpd([91, 122, 0, 46.8, 0], spd=5000)
robot.light(True)
for picnum in range(0, duration):
	try:
		if picnum == 0:
			img = robot.captureImage()
			picname = str(picnum) + '.png'
			cv2.imwrite(picname, img)
		else:
			img2 = robot.captureImage()
			picname = str(picnum) + '.png'
			cv2.imwrite(picname, img2)
	        image1 = cv2.resize(img, (1280, 960))
	        h1, s1, v1 = cv2.split(cv2.cvtColor(image1, cv2.COLOR_BGR2HSV))
	        image2 = cv2.resize(img2, (1280, 960))
	        h2, s2, v2 = cv2.split(cv2.cvtColor(image2, cv2.COLOR_BGR2HSV))
	        image = cv2.subtract(v1,v2)
	        ret,gray = cv2.threshold(image,25,255,0)
	        gray2 = gray.copy()
	        gray2 = cv2.morphologyEx(gray2, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
	        gray2 = cv2.Canny(gray2,30,100)
	        gray2 = np.nonzero(gray2)
	        gray2 = len(np.nonzero(gray2)[0])
	        img = img2
	        cv2.imwrite('delta_' + str(picnum) + '.png', gray2)
	except:
		print picnum
		robot = robotutil.MAPLE("MAPLE.cfg")
robot.light(False)