## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: robotutil.py
#  Description: Contains classes and functions used to control
#  the FlySorter automated experiment platform (project name MAPLE).
#  High-level commands can be called in primary experimental scripts with relevant coordinates.

## Dependencies
import os
import math
import cv2
import numpy as np
import time
import ConfigParser
import random as rand
import pyicic.IC_ImagingControl
import flysorterSerial

class santaFe:
    """Class for fly manipulation robot."""

    smoothiePort = ""
    dispenserPort = ""

    # These variables should be kept up-to-date by functions that change them
    currentPosition = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    currentRotation = np.array([0.0, 0.0])

    # Initialize these variables to zero -- they should be read in by readConfig
    Z0Offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    Z2Offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    maxExtents = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    OutputDir = ""

    isInitialized = False

    travelSpeed = 5000
    acceleration = 200

    # Configuration defaults
    configDefaults = {'WorkspaceXSize': '1000',
                      'WorkspaceYSize': '270',
                      'Z1FloorHeight': '40.0',
                      'MaxZ0Depth': '68',
                      'MaxZ1Depth': '50',
                      'MaxZ2Depth': '55',
                      'OutputDir': 'Photos',
                      'Z0OffsetX': '-40',
                      'Z0OffsetY': '0',
                      'Z0OffsetZ': '23',
                      'Z2OffsetX': '40',
                      'Z2OffsetY': '0',
                      'Z2OffsetZ': '8',
                      'HFOV': '14.5',
                      'VFOV': '11.25'
                      }

    def __init__(self, robotConfigFile):
        print "Reading config file...",
        self.config = ConfigParser.RawConfigParser(self.configDefaults)
        self.readConfig(robotConfigFile)
        print "done."

        print "Initializing serial connections:"
        self.smoothie = None

        # Search serial ports, look for motor control board and smoothie
        portList = flysorterSerial.availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            # print "Trying port", portDesc
            tempPort = flysorterSerial.serialDevice(portDesc, 115200)
            if tempPort.sendCmdGetReply("version\n").startswith("Build version"):
                print "Port:", portDesc, "is smoothie."
                self.smoothiePort = portDesc
                self.smoothie = tempPort
                portList.remove(portDesc)
                continue
            tempPort.close()

        if self.smoothie is None:
            print "Serial initialization failed."
            if self.smoothie is None:
                print "Smoothie board not found."
            return

        print "Initializing camera...",
        self.cam = cameraInit()
        if self.cam == None:
            print "Camera init fail."
            self.smoothie.close()
            return

        print "done."

        self.currentPosition = self.getCurrentPosition()
        self.isInitialized = True
        print "Robot initialized."
        urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=1')
        return

    # Read in the config, and assign values to the appropriate vars
    def readConfig(self, configFile):
        self.config.read(configFile)
        self.OutputDir = self.config.get('DEFAULT', 'OutputDir')
        self.Z1FloorHeight = float(self.config.get('DEFAULT', 'Z1FloorHeight'))
        self.maxExtents = np.array( [ float(self.config.get('DEFAULT', 'WorkspaceXSize')), float(self.config.get('DEFAULT', 'WorkspaceYSize')),
                                      float(self.config.get('DEFAULT', 'MaxZ0Depth')),
                                      float(self.config.get('DEFAULT', 'MaxZ1Depth')),
                                      float(self.config.get('DEFAULT', 'MaxZ2Depth')) ] )
        self.Z0Offset = np.array( [ float(self.config.get('DEFAULT', 'Z0OffsetX')), float(self.config.get('DEFAULT', 'Z0OffsetY')),
                                      float(self.config.get('DEFAULT', 'Z0OffsetZ')), 0.0, 0.0 ] )
        self.Z2Offset = np.array( [ float(self.config.get('DEFAULT', 'Z2OffsetX')), float(self.config.get('DEFAULT', 'Z2OffsetY')),
                                      0.0, 0.0, float(self.config.get('DEFAULT', 'Z2OffsetZ')) ] )
        self.FOV = np.array([ float(self.config.get('DEFAULT', 'HFOV')), float(self.config.get('DEFAULT', 'VFOV')) ])
        return

    # Resets all solenoid valves, camera, and light circle
    def release(self):
        self.light(False)
        self.flyManipVac(False)
        self.smallPartManipVac(False)
        self.flyManipAir(False)
        self.smallPartManipAir(False)
        self.cam.close()
        self.smoothie.close()

    # Homing via g-code command (moves along axes according to .ini file uploaded to robot)
    def home(self):
        self.smoothie.sendSyncCmd("G28\n")
        self.smoothie.sendSyncCmd("G01 F{0}\n".format(self.travelSpeed))
        self.currentPosition = np.array([0., 0., 0., 0., 0.])
        return

    # Only homes end effectors; Faster than regular home()
    def homeZ(self):
    	self.smoothie.sendSyncCmd("G28 B0\n")
    	return

	# Captures current camera image; returns as numpy array
    def captureImage(self):
        self.cam.start_live()
        self.cam.snap_image()
        (imgdata, w, h, d) = self.cam.get_image_data()
        self.cam.stop_live()
        img = np.ndarray(buffer = imgdata,
                         dtype = np.uint8,
                         shape = (h, w, d))
        return img

   	# Returns current position as array
    def getCurrentPosition(self):
        # M114.2 returns string like: "ok MCS: X:0.0000 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000"
		while True:
			try:
				positions = self.smoothie.sendCmdGetReply("M114.2\n").split(' ')
				xPos  = float(positions[2].split(':')[1])
				break
			except:
				print 'Error: position temporarily lost'
				self.dwell(10)
		xPos  = float(positions[2].split(':')[1])
		yPos  = float(positions[3].split(':')[1])
		z0Pos = float(positions[4].split(':')[1])
		z1Pos = float(positions[5].split(':')[1])
		z2Pos = float(positions[6].split(':')[1])
		return np.array( [ xPos, yPos, z0Pos, z1Pos, z2Pos ] )

	# Simple coordinate-move command
    def moveXY(self, pt):
        if (len(pt) != 2):
            print 'Error: incorrect coordinate string. Dropping moveXY instruction'
            return
        if ( self.isPtInBounds((pt[0], pt[1], 0., 0., 0.)) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return
        cmd = "G01 X{0[0]} Y{0[1]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[0] = pt[0]
        self.currentPosition[1] = pt[1]

    # Coordinate-move command with added mandatory speed (Also updates default speed)
    def moveXYSpd(self, pt):
        if (len(pt) != 3):
            print 'Error: missing speed parameter'
            return

        if ( self.isPtInBounds((pt[0], pt[1], 0., 0., 0.)) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return

        cmd = "G01 X{0[0]} Y{0[1]} F{0[2]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[0] = pt[0]
        self.currentPosition[1] = pt[1]

    def moveCirc(self, r, n, endpos, spd, rest, full=False):        # use moveCirc2 whenever possible (smoother movement)
    ## circular motion around current point; r: radius, n: amount of points (fine-ness),
    ## endpos: movement end point in multiples of pi, spd; speed, rest: time between movements (fine-ness)
        self.dwell(t=500) # to get accurate starting point
        curcircle = np.arange(n*2).reshape((n, 2))
        XYPos = self.getCurrentPosition()
        CircStart = XYPos[0:2]
        tempPos = XYPos[0:2]
        tempPosSpd = [0,1,2]
        for x in range(0,n):
            curcircle[x,:] = math.sin(endpos*math.pi/n*x)*r, math.cos(endpos*math.pi/n*x)*r
            changeX = (curcircle[x,0] - curcircle[x-1,0])
            changeY = (curcircle[x,1] - curcircle[x-1,1])
            if x <= 2:  # Removes large first .sin change in X pos
                tempPos = tempPos
            else:
                tempPos[0] += changeX
                tempPos[1] += changeY
            tempPosSpd[0] = tempPos[0]
            tempPosSpd[1] = tempPos[1]
            tempPosSpd[2] = spd
            if x >= n-1:
                tempPosSpd[2] = self.travelSpeed   # resets speed (carries over in future commands)
            self.moveXYSpd(pt=tempPosSpd)
            self.dwell(rest)
        CircEnd = tempPos
        if full!= False:
            self.moveXY(pt=CircStart)
        self.dwell(t=200)   # ensures plate is still from current movement
        return CircEnd


    # Circular movement followed by lowering of fly manipulating end effector (Hit-detection causes Z-axis retreat)
    def moveCirc2(self, mid, r, n=360, startpos=0, endpos=360, spd=1500, rest=100, z=53, full=True, retreatZ=10, descendZ=9):       #also lowers onto z height
        careful = 0     # failsafe when already starts lowered
        if spd >= 2001:
            print 'Adjusting speed to safe level 2000 ...'	# Prevents calibration errors due to post-motion-induced positional changes.
            spd = 2000
        while startpos > 360:
            startpos = startpos - 360
            print 'Startpos corrected to:', startpos
        while endpos > 360:
            endpos = endpos - 360
            print 'Endpos corrected to:', endpos
        deg = np.ones((n+1,3))
        for x in range(0,len(deg)):
            deg[x,:] = (math.sin(2*math.pi/n*x)*r)+mid[0], (math.cos(2*math.pi/n*x)*r)+mid[1], spd
        if endpos - startpos < 0:
            step = -1
            offset = -2
        else:
            step = 1
            offset = 2
        for i in range(startpos, endpos+1, step):       # loop that performs the circular movement and lowering only on iteration 1
            if careful != 1:
                if step == 1 and i >= endpos-offset:
                    deg[:,2] = self.travelSpeed     # resets slow speed to config normal
                elif step == -1 and i <= endpos-offset:
                    deg[:,2] = self.travelSpeed
                self.moveXYSpd(pt=deg[i,:])
                self.dwell(rest)
                if i == startpos:
                    self.dwell(10)  # buffer before lowering
                XYpos = self.getCurrentPosition()
                if i == startpos and (XYpos[0] - deg[startpos,0] <= 1) and (XYpos[1] - deg[startpos,1] <= 1):   # only lower end effector on first iteration and if degrees match
                    self.dwell(10)
                    self.moveZ(pt=[XYpos[0],XYpos[1],0,0,z-descendZ]) # lowers to descendZ units above detected opening at height z
                    self.dwell(1) # buffer before loop
                    for j in range(1, descendZ+2):     #carefully lower into opening and start at 0 just to also check current limit
                        self.dwell(1)
                        self.moveZ(pt=[XYpos[0],XYpos[1],0,0,(z-descendZ)+j])
                        careful = self.getLimit()
                        if careful == 1:
                            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                            break
            elif careful != 0:
                self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                break
        if careful != 1 and full == True:
            self.moveXYSpd(pt=deg[endpos,:])
            XYpos = deg[endpos,0:2]
            self.dwell(1)
        elif careful != 0:
            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
        return {'endXY':XYpos, 'endDeg':endpos, 'oldMid': mid, 'limit': careful, 'startDeg': startpos}


    # Uses repeated hit-detection of moveCirc2() as error-correction of slightly deviating opening-detection
    def tryOpening(self, mid, r, n=360, startpos=0, endpos=360, spd=1000, rest=5, z=53, full=True, retreatZ=10, descendZ=9):
        tryspd = spd
        trymid = mid
        trystart = startpos
        tryend = endpos
        tryz = z
        unsure=0
        radi = r
        careonce = 0
        trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=trystart, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
        startposFirst = startpos
        while trylower['limit'] == 1 and unsure != 1:
            for cw in xrange(2,10,2):
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos + cw
                    trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            startpos = startposFirst
            for ccw in xrange(2,10,2):       # can skip 0 degree offset due to clockwise motion
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos - ccw
                    trylower = self.moveCirc2(mid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            if trylower['limit'] == 1:
                print 'Could not find opening - detecting anew...'
                unsure = 1
                self.homeZ()
        trylower.update({'limitonce':careonce})
        return trylower

    # Step-by-step lowering of fly-manipulating end effector with hit-detection
    def lowerCare(self, z, descendZ=9, retreatZ=18):		# z: depth of descend; descendZ: number of careful steps to reach depth z; retreatZ: RELATIVE height retreat upon hit
		if z > 55 or z < 0:
			print 'Z not in range 0,55 - skipping...'
			return
		if descendZ > z:
			print 'descendZ larger than Z - correcting...'
			descendZ = z-1
		posbegin = self.getCurrentPosition()
		self.moveZ(pt=[posbegin[0],posbegin[1],0,0,z-descendZ])
		for i in range(1, descendZ+2):
			self.dwell(1)
			self.moveRel(pt=[0,0,0,0,1])
			careful = self.getLimit()
			if careful == 1:
				self.moveRel(pt=[0,0,0,0,-retreatZ])
				break
		posend = self.getCurrentPosition
		return {'pos.begin': posbegin, 'pos.end': posend, 'limit': careful}

	# Highest order command to withdraw a single fly from a single well in the housing module
    def homeWithdraw(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, dislodgeZ=10, vacBurst=1, vacDur=4000, homeZ=45):
        if refptX != 'N':
	        self.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
	        self.dwell(t=1)
        self.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, dislodgeZ, 5000])        # Go to actual home
        self.dwell(t=1)
        self.flyManipAir(True)
        trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
        if trylowerHome['limit'] == 0:
            self.dwell(t=1)
            for b in range(0,vacBurst):
            	self.flyManipAir(False)
                self.smallPartManipVac(True)
                self.dwell(t=rand.choice(range(2,4)))
                self.smallPartManipVac(False)
                self.dwell(t=rand.choice(range(2,4)))
            self.smallPartManipVac(True)
            self.dwell(t=vacDur)
            self.moveRel(pt=[0, 0, 0, 0, -homeZ])
            self.dwell(t=10)
        else:
        	self.flyManipAir(False)
        	self.home()
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

    # Highest order command to deposit a single fly in a well in the housing module
    def homeDeposit(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44):
    	if refptX != 'N':
	        self.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
	        self.dwell(t=1)
        self.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, 10, 5000])        # Go to actual home
        self.dwell(t=1)
        trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
        if trylowerHome['limit'] == 0:
            self.dwell(t=1)
            self.smallPartManipVac(False)
            for b in range(0,vacBurst):
                self.flyManipAir(True)
                self.dwell(t=rand.choice(range(5,6)))
                self.flyManipAir(False)
                self.dwell(t=rand.choice(range(5,6)))
            self.dwell(t=50)
            self.moveRel(pt=[0, 0, 0, 0, -homeZ])
            self.dwell(t=10)
        else:
        	self.home()
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

    # Highest order command to withdraw a single fly from a single arena in the behavioral module (Different withdraw-strategies accessible using vacstrategy)
    def arenaWithdraw(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, vacPos, vacZ, closePos, vacstrategy=2, vacBurst=1, imgshow=0):
    	strategy = vacstrategy
    	missonce = 0
    	print 'Using strategy', strategy
        self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
        self.dwell(t=1)
        degs1 = int(self.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
        self.dwell(t=5)
        self.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
        self.dwell(t=10)
        Mid1 = self.getCurrentPosition()
        self.dwell(1)
        endpos1 = vacPos
        if strategy == 3 or strategy == 6:
        	self.smallPartManipVac(True)
        tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
        miss = tempCoord['limit']
        missonce = missonce + tempCoord['limitonce']
        self.dwell(50)
        if miss == 1:
            print 'Possible misalignment - resetting...'
            self.home()
            return {'miss': miss, 'missonce': missonce}
        elif miss != 1:
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            if strategy == 1:
	            self.smallPartManipVac(False)
	            for b in range(0,vacBurst):
	                self.smallPartManipVac(True)
	                self.dwell(t=200)
	                self.smallPartManipVac(False)
	                self.dwell(t=10)
	            self.smallPartManipVac(True)
	            self.dwell(t=400)
	            self.smallPartManipVac(False)
	            self.dwell(t=5)
	            self.smallPartManipVac(True)
            elif strategy == 2:
            	self.smallPartManipVac(False)
                self.flyManipAir(True)
                self.dwell(t=5)
                self.flyManipAir(False)
                self.dwell(t=5)
                self.flyManipAir(True)
                self.dwell(t=5)
                self.flyManipAir(False)
                self.dwell(t=5)
                self.smallPartManipVac(True)
                self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ+1])
                self.dwell(t=50)
                self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ-1])
                self.dwell(t=50)
                self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                self.dwell(t=50)
            elif strategy == 3:
            	sweeppos1 = tempCoord['endDeg']
            	if sweeppos1 < 180:
            		sweeppos2 = 140
            	else:
            		sweeppos2 = 220
            	checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
            	missonce = missonce + checkformiss['limitonce']
            	self.dwell(50)
            	checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
            	missonce = missonce + checkformiss['limitonce']
            elif strategy == 4:
                self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
                self.dwell(t=10)
                self.smallPartManipVac(True)
                self.dwell(t=20000)
            elif strategy == 5:
            	self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            	self.dwell(t=5)
            	self.flyManipAir(True)
            	self.dwell(t=500)
            	self.flyManipAir(False)
            	self.smallPartManipVac(True)
            	self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            	self.dwell(t=1000)
            elif strategy == 6:
            	sweeppos1 = tempCoord['endDeg']
            	if sweeppos1 < 180:
            		sweeppos2 = 140
            	else:
            		sweeppos2 = 220
            	self.flyManipAir(True)
            	self.dwell(t=5)
            	self.flyManipAir(False)
            	self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
            	self.flyManipAir(True)
            	self.dwell(t=5)
            	self.flyManipAir(False)
            	self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
            	self.dwell(50)
            endpos2 = closePos
            tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
            missonce = missonce + tempCoord['limitonce']
            endpos1 = tempCoord['endDeg']
            self.dwell(10)
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
            self.dwell(10)
            return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

    # Highest order command to deposit a single fly in a single arena in the behavioral module
    def arenaDeposit(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, airPos, airZ, closePos, airBurst=1, imgshow=0):
    	missonce = 0
    	self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
        self.dwell(t=1)
        degs1 = int(self.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
        self.dwell(t=5)
        self.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
        self.dwell(t=10)
        Mid1 = self.getCurrentPosition()
        self.dwell(1)
        endpos1 = airPos
        tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
        miss = tempCoord['limit']
        missonce = missonce + tempCoord['limitonce']
        self.dwell(50)
        if miss == 1:
            print 'Possible misalignment - full reset...'
            self.home()
            return {'miss': miss, 'missonce': missonce}
        elif miss != 1:
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,airZ])
            self.smallPartManipVac(False)
            self.dwell(t=5)
            for b in range(0,airBurst):
                self.flyManipAir(True)
                self.dwell(t=rand.choice(range(5,6)))
                self.flyManipAir(False)
                self.dwell(t=rand.choice(range(5,6)))
            self.dwell(t=50)
            endpos2 = closePos
            tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
            missonce = missonce + tempCoord['limitonce']
            endpos1 = tempCoord['endDeg']
            self.dwell(50)
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
            self.dwell(10)
            return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

    # Moves robot to dispenser location, sends command to dispend fly, tries dispiter times to vacuum fly out
    def dispenseWithdraw(self, dispX, dispY, dispZ, dispiter=2, onlyifsure=1):
    	dispsuccess = 0
    	self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    	self.dwell(50)
    	limit = self.lowerCare(dispZ, descendZ=6, retreatZ=12)
    	if onlyifsure == 0:
    		print 'Depositing even if fly may be stuck.'
    	if limit != 1:
        	self.smallPartManipVac(True)
        	for iter in range(dispiter):
        		dispsuccess = self.dispenseFly()
        		if dispsuccess == 1:
        			print 'Fly dispensed.'
        			self.dwell(2000)
        			self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
        			return dispsuccess
        		elif dispsuccess == 0:
        			self.smallPartManipVac(False)
        		elif dispsuccess == 2 and onlyifsure == 1:
        			self.smallPartManipVac(False)
        		elif dispsuccess == 2 and onlyifsure == 0:
        			print 'Quantum fly in tunnel - better safe than sorry protocol commencing.'
        			self.dwell(3000)
        			self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
        elif limit == 1:
        	print 'Possible misallignment. Check fly dispenser.'
        	self.home()
        	return None
        self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    	return dispsuccess

    # Tries to dispense and deposit all newly hatched flies into home plate / saves how many flies were successfully dispensed for longer processing
    def dispenseHIfHatched(self, homecoordX, homecoordY, dispX, dispY, dispZ, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, maxconsecstuck=4):
        nDispensed = carryovernDispensed
        IsHatched = 1
        consecStuck = 0
        while IsHatched == 1 or (IsHatched == 2 and onlyifsure == 0) and not consecStuck > maxconsecstuck:
            IsHatched = self.dispenseWithdraw(dispX=dispX, dispY=dispY, dispZ=dispZ, dispiter=dispiter, onlyifsure=onlyifsure)
            if IsHatched == 1:
                self.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
                nDispensed = nDispensed+1
            elif IsHatched == 2 and onlyifsure == 0:
                self.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
                nDispensed = nDispensed +1
                consecStuck = consecStuck +1
            else:
                break
                print 'No fly dispensed.'
        print 'Dispensed', nDispensed, 'flies total.'
        return nDispensed

    # Repeatedly collect successfully dispensed flies (newly hatched) and deposit into consecutive single wells in the housing module
    # Not accurate loop time-wise but ensures a minimum amount of time tried collecting. All times in ms.
    def collectHatchedForT(self, homecoordX, homecoordY, dispX, dispY, dispZ, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, collectT=3600, collectInt=60, maxconsecstuck=4):
        for n in range(collectT/collectInt):
            carryovernDispensed = self.dispenseHIfHatched(homecoordX=homecoordX, homecoordY=homecoordY, dispX=dispX, dispY=dispY, dispZ=dispZ, onlyifsure=onlyifsure, carefulZ=9, vacBurst=vacBurst, homeZ=homeZ, dispiter=dispiter, carryovernDispensed=carryovernDispensed, maxconsecstuck=maxconsecstuck)
            time.sleep(collectInt)

    # Moves to coordinates and returns whether movement was detected
    def detectFlyInArena(self, camcoordX, camcoordY, camcoordZ):
        self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
        self.dwell(10)
        flyremaining = self.detectFly( minpx=40, maxpx=2000)
        return flyremaining

    # Returns limit pin state (Used in hit detection)
    def getLimit(self):
        limitgot = 0
        while limitgot < 10:
            try:
                templimit = str(self.smoothie.sendCmdGetReply("M119\n").split(' '))
                limit = int(templimit[150])
                limitgot = 10
            except:
                limitgot = limitgot + 1
        if limit == 1:
            print 'Limit is', limit, '!'
        return limit

	# Simple Z-axis move command. First 2 inputs (X and Y axis) should be 0.
    def moveZ(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string. Dropping moveZ instruction'
            return
        if ( self.isPtInBounds(pt) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return
        cmd = "G01 Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell(1)
        self.currentPosition = pt
        return

    # Complex move command. All axes and end effectors can be moved in unison.
    def moveTo(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string:', pt, ' Dropping moveTo instruction'
            return
        if ( self.isPtInBounds(pt) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return
        cmd = "G01 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell(1)
        self.currentPosition = pt
        return

    # Complex move command with added speed parameter (Updates default speed)
    def moveToSpd(self, pt):
        if (len(pt) != 6):
            print 'Error: incorrect coordinate string. Dropping moveTo instruction'
            return
        cord = pt[0:5]
        if ( self.isPtInBounds(cord) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return
        cmd = "G01 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]} F{0[5]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell(1)
        self.currentPosition = pt
        return

    # Moves robot relative to current position, not absolute coordinates.
    def moveRel(self, pt):
        self.moveTo( map(sum,zip(self.currentPosition, pt)) )

    # Moves robot to all coordinates in the list (Needs 5 scalars per list entry)
    def moveXYList(self, ptList):
        for pt in ptList:
            self.moveXY(pt)
        return

    # Check whether desired coordinate (vector of exactly len 5) is larger than maximum workspace size or smaller than 0
    def isPtInBounds(self, pt):
        if ( len(pt) != 5 ):
            print 'Error: incorrect coordinate length.'
            return False
        if (  ( pt[0] > self.maxExtents[0] ) or
              ( pt[1] > self.maxExtents[1] ) or
              ( pt[2] > self.maxExtents[2] ) or
              ( pt[3] > self.maxExtents[3] ) or
              ( pt[4] > self.maxExtents[4] ) or
              ( pt[0] < 0.0 ) or ( pt[1] < 0.0 ) or ( pt[2] < 0.0 ) or ( pt[3] < 0.0 ) or ( pt[4] < 0.0 ) ):
            return False
        else:
            return True

    # Wait until last robot action (movement) is completed
    def dwell(self, t):
		while True:
			try:
				cmd = "G04 P{0}\n".format(t)
				self.smoothie.sendSyncCmd(cmd)
				break
			except:
				print 'Error: retrying to send dwell command'
		return

	# Controls light-circle LED around camera
    def light(self, onOff = False):
        if ( onOff == True ):
            cmd = "M48\n"
        else:
            cmd = "M49\n"
        self.smoothie.sendCmd(cmd)
        return

    # Controls positive air pressure out of fly manipulating end effector
    def flyManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M46\n"
        else:
            cmd = "M47\n"
        self.smoothie.sendCmd(cmd)
        return

    # Controls negative air pressure out of fly manipulating end effector
    def flyManipVac(self, onOff = False):		# rerouted pins to smallPart vacuum
        if (onOff == True):
            cmd = "M44\n"
        else:
            cmd = "M45\n"
        self.smoothie.sendCmd(cmd)
        return

    # Controls positive air pressure out of part manipulating end effector (Releases part)
    def smallPartManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M42\n"
        else:
            cmd = "M43\n"
        self.smoothie.sendCmd(cmd)
        return

    # Controls negative air pressure out of part manipulating end effector (Holds part)
    def smallPartManipVac(self, onOff = False):		# rerouted to fly vacuum
        if (onOff == True):
            cmd = "M40\n"
        else:
            cmd = "M41\n"
        self.smoothie.sendCmd(cmd)
        return

    # Finds immobile fly on white surface (CO2 board)
	def findFly(self, image):
        # Convert BGR to HSV
		h, s, v = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        # Now threshold in the value channel
		r, mask = cv2.threshold(255-v, 100, 255, cv2.THRESH_BINARY)
        # Find contours
		contours, h = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		for c in contours:
			mmnts = cv2.moments(c)
			if ( 20000 < mmnts['m00'] < 200000 ):
                # Center of contour is m10/m00, m01/m00
				(pxHeight, pxWidth) = mask.shape
				imgCoords = np.array([ int(mmnts['m10'] / mmnts['m00'] ), int( mmnts['m01'] / mmnts['m00']) ], dtype=np.int16)
				cv2.line(image, tuple(imgCoords - np.array([ 20, 0])), tuple(imgCoords + np.array([ 20, 0])), (0,0,0), 5)
				cv2.line(image, tuple(imgCoords - np.array([ 0, 20])), tuple(imgCoords + np.array([ 0, 20])), (0,0,0), 5)
				cv2.line(image, tuple(imgCoords - np.array([ 20, 0])), tuple(imgCoords + np.array([ 20, 0])), (255,255,255), 3)
				cv2.line(image, tuple(imgCoords - np.array([ 0, 20])), tuple(imgCoords + np.array([ 0, 20])), (255,255,255), 3)
				coords = np.array([ (mmnts['m10'] / mmnts['m00'] - pxWidth/2.)*self.FOV[0]/pxWidth,
                                    (mmnts['m01'] / mmnts['m00'] - pxHeight/2.)*self.FOV[1]/pxHeight ])
				return coords
		return None

	# Compares 2 images ~800ms apart and returns 1 if minpix < detected difference pixels < maxpix
    def detectFly(self, minpx=40, maxpx=800,showimg=False):
		self.light(True)
		time.sleep(0.2)
		self.light(True)
		time.sleep(0.2)
        image1 = self.captureImage()
		self.dwell(800)
        image2 = self.captureImage()
		self.light(False)
		image1 = cv2.resize(image1, (1280, 960))
		h1, s1, v1 = cv2.split(cv2.cvtColor(image1, cv2.COLOR_BGR2HSV))
		image2 = cv2.resize(image2, (1280, 960))
		h2, s2, v2 = cv2.split(cv2.cvtColor(image2, cv2.COLOR_BGR2HSV))
		image = cv2.subtract(v1,v2)
		ret,gray = cv2.threshold(image,25,255,0)
		gray2 = gray.copy()
		gray2 = cv2.morphologyEx(gray2, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
		gray2 = cv2.Canny(gray2,30,100)
		gray2 = np.nonzero(gray2)
		gray2 = len(np.nonzero(gray2)[0])
		print 'detected', gray2, 'moving pixels.'
		if showimg == True:
			cv2.imshow('image', gray2)
			cv2.waitKey(0)
		if minpx<gray2<maxpx:
			return True
		else:
			return False

	# Input coordinate vector is returned as logic depending on iterating detectFly() on each index
    def sweep(self, ptsx, ptsy, camz=45, spd=5000):
		detectvect = range(len(ptsx))
		indvect = np.array(range(len(ptsx)))
		for i in range(len(ptsx)):
			self.moveToSpd(pt=[float(ptsx[i]), float(ptsy[i]), 0, camz, 0, spd])
			detectvect[i] = self.detectFly()
		detectvect = np.array(detectvect, dtype = bool)
		indvect = np.array(indvect[detectvect])
		return indvect

	# Finds circle in input image. Used to find opening in arena lid to allow access.
    def findOpening(self, image, slowmode=False, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0):
        result = []
        startp1 = startp1
        startp2 = startp2
        startp3 = startp3
        imgshow=imgshow
        detect = 0
        if slowmode == False:
            image = cv2.imread(image)
            image = cv2.resize(image, (1280, 960))
            output = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            while detect == 0:      # decrease sensitivity until at least one circle is found
                circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
                if  circles is not None:
                    # convert the (x, y) coordinates and radius of the circles to integers
                    circles = np.round(circles[0, :]).astype("int")
                    #print 'detected', len(circles), 'circles'
                    # loop over the (x, y) coordinates and radius of the circles
                    for i in range(0,len(circles)):
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(output, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
                        cv2.rectangle(output, (circles[i,0] - 5, circles[i,1] - 5), (circles[i,0] + 5, circles[i,1] + 5), (0, 128, 255), -1)
                    if len(circles) == 1:
                        detect = 1
                    elif startp2 > 100 and len(circles) > 1:
                        startp2 = startp2 - 3
                    elif startp2 <= 100 or len(circles) > 1:
                        detect = 1      # to leave loop if too many circles get found repeatedly -- unlikely to result in wrong angle as it needs validation
                else:
                    startp2 = startp2 - 3       # decrease sensitivity if no circles were found
            if imgshow == 1:
            	cv2.imshow("thresh", thresh)
            	cv2.imshow("output", output)
            	cv2.waitKey(0)
        elif slowmode == True:      # Improves findOpening on the off-chance that something wrong in the image processing
            oldcircles = np.zeros((1,3), dtype=np.int)
            detect = 0
            while detect == 0:      # decrease sensitivity until at least one circle is found
                image2 = cv2.imread(image)
                image3 = cv2.resize(image2, (1280, 960))
                output = image3.copy()
                gray = cv2.cvtColor(image3, cv2.COLOR_BGR2GRAY)
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    if len(circles) == 1:
                        print 'Detected ',len(circles), 'circle.'
                    else:
                        print 'Detected ',len(circles), 'circles.'
                    detect = 1
                    # show the output image
                    for i in range(0,len(circles)):
                     # draw the circle in the output image, then draw a rectangle
                     # corresponding to the center of the circle
                        if imgshow == 1:
                            cv2.circle(output, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
                            cv2.rectangle(output, (circles[i,0] - 5, circles[i,1] - 5), (circles[i,0] + 5, circles[i,1] + 5), (0, 128, 255), -1)
                            cv2.imshow("thresh", thresh)
                            cv2.imshow("output", output)
                            cv2.waitKey(0)
                else:
                    startp2 = startp2 - 3       # decrease sensitivity if no circles were found
        return circles

    # Returns degrees of a point's coordinates relative to the image midpoint (the opening)
    def getDegs(self, img, img_width=1280, img_height=960):
        imgmid = [img_width/2, img_height/2]
        if len(img[:,1]) >= 2 and not (img[0,0] - img[1,0] >= 150) and not (img[0,1] - img[1,1] >= 150):    # allows 2 close circles and takes mean coords instead (Accuracy - Iteration tradeoff. Works well if less than 10px apart).
            img[0,0] = (img[0,0] + img[1,0])/2
            img[0,1] = (img[0,1] + img[1,1])/2
            print 'Multiple adjoining openings detected - correcting target coordinates...'
        dx = img[0,0] - imgmid[0]
        dy = (img_height - img[0,1]) - imgmid[1]
        rads = math.atan2(-dy,dx)
        rads %= 2*math.pi
        degs = math.degrees(rads)
        degs = (degs-90)* (-1)
        if degs <= 0:       # converts degrees
            degs = 360 + degs
        if degs <= 180:
            degs = (degs - 180) * (-1)
        elif degs >= 180 and degs <= 360:
            degs = ((degs - 360) * (-1)) + 180
        return degs

    # Combines findOpening and getDegs
    def findDegs(self, slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=139, startp2=150, startp3=2.6, imgshow=0):
    	trymax=MAX_SIZE
    	trymin=MIN_SIZE
    	try1 = startp1
    	try2 = startp2
    	try3 = startp3
    	imgshow = imgshow
    	slwmd = slowmode
        if slowmode == True:
            certain = 0
            while certain != 1:
                tempdeg = np.arange(2)
                for i in range(0,2):
                    self.light(True)
                    time.sleep(0.2)
                    self.cam.start_live()
                    ICexcept = 0
                    while ICexcept < 5:
	                    try:
	                        self.cam.snap_image()
	                        ICexcept = 5
	                    except:
	                        ICexcept = ICexcept +1
	                        print 'Error: Could not snap picture - retrying...'
                    self.cam.save_image(''.join(['curImage.jpg']), 1, jpeq_quality=100)
                    self.cam.stop_live()
                    self.light(False)
                    img = self.findOpening('curImage.jpg', slowmode=slwmd, MAX_SIZE=trymax, MIN_SIZE=trymin, startp1=try1, startp2=try2, startp3=try3, imgshow=imgshow)
                    tempdeg[i] = self.getDegs(img)
                if abs(tempdeg[0] - tempdeg[1]) <= precision:
                    certain = 1
                    return np.mean(tempdeg)
        elif slowmode == False:
            self.light(True)
            time.sleep(0.2)
            self.cam.start_live()
            self.cam.snap_image()
            self.cam.save_image(''.join(['curImage.jpg']), 1, jpeq_quality=100)
            self.cam.stop_live()
            self.light(False)
            img = self.findOpening('curImage.jpg', slowmode=slwmd, MAX_SIZE=trymax, MIN_SIZE=trymin, startp1=try1, startp2=try2, startp3=try3, imgshow=imgshow)
            tempdeg = self.getDegs(img)
            return tempdeg


# Set up close-up camera
def cameraInit():
    global ic_ic
    print "Opening camera interface."
    ic_ic = pyicic.IC_ImagingControl.IC_ImagingControl()
    ic_ic.init_library()
    cam_names = ic_ic.get_unique_device_names()
    cam = ic_ic.get_device(cam_names[0])
    cam.open()
    cam.gain.value = 10
    cam.exposure.auto = False
    cam.exposure.value = -7
    cam.set_video_format('BY8 (2592x1944)')
    cam.set_frame_rate(4.00)
    cam.prepare_live()
    return cam
