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
import sys
import traceback
import glob
import serial
import time
import ConfigParser
import random as rand
import smtplib
import email
import poplib
from email import parser
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
import urllib2
import pyicic.IC_ImagingControl


# Serial communications class that is used for multiple devices.
# In MAPLE, the smoothie board is a serial devices. The smoothie is connected directly
# via USB.
#
# 1. Smoothieboard - controls X, Y, Z0, Z1 and Z2 (X, Y, Z, A and B axes, respectively)
#
# Documentation is available online for G-codes (smoothie):
#
# http://smoothieware.org/supported-g-codes and
# http://reprap.org/wiki/G-code
#
# This class is also used to communicate with the Fly Dispenser (if connected & used).
# The Dispenser, similar to the Smoothie, connects by USB and appears as a COM port.


class serialDevice:
    """Serial class for generic serial device."""

    WaitTimeout = 3
    portName = ""

    def __init__(self, port, baud = 9600, timeout = float(0.1)):
        self.isOpened = False
        try:
            self.ser = serial.Serial(port, baudrate = baud, timeout = timeout)
        except:
            print "Failed to open port", port
            return
        self.isOpened = True

    def close(self):
        self.ser.close()

    # Retrieve any waiting data on the port
    def getSerOutput(self):
        #print "GSO:"
        output = ''
        while True:
            # read() blocks for the timeout set above *if* there is nothing to read
            #   otherwise it returns immediately
            byte = self.ser.read(1)
            if byte is None or byte == '':
                break
            output += byte
            if byte == '\n':
                break
        #print "GSO Output:", output
        return output

    # Block and wait for the device to reply with "ok" or "OK"
    # Times out after self.WaitTimeout (set above)
    def waitForOK(self):
        #print "WFO:"
        output = ''
        timeoutMax = self.WaitTimeout / self.ser.timeout
        timeoutCount = 0
        while True:
            byte = self.ser.read(1)
            if byte is None or byte == '':
                timeoutCount += 1
                time.sleep(1)
            else:
                output += byte
            if timeoutCount > timeoutMax:
                print 'Serial timeout.'
                break
            if byte == '\n':
                break
        if ( not output.startswith("ok") ) and ( not output.startswith("OK") ):
            print "Unexpected serial output:", output.rstrip('\r\n'), "(", ':'.join(x.encode('hex') for x in output), ")"

    # Send a command to the device via serial port
    # Asynchronous by default - doesn't wait for reply
    def sendCmd(self, cmd):
        #print "SC:", cmd
        self.ser.write(cmd)
        self.ser.flush()

    # Send a command to the device via serial port
    # Waits to receive reply of "ok" or "OK" via waitForOK()
    def sendSyncCmd(self, cmd):
        #print "SSC:", cmd
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        self.waitForOK()

    # Send a command and retrieve the reply
    def sendCmdGetReply(self, cmd):
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        return self.getSerOutput()


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
        portList = availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            # print "Trying port", portDesc
            tempPort = serialDevice(portDesc, 115200)
            if tempPort.sendCmdGetReply("version\n").startswith("Build version"):
                print "Port:", portDesc, "is smoothie."
                self.smoothiePort = portDesc
                self.smoothie = tempPort
                portList.remove(portDesc)
                continue
            tempPort.close()

        for portDesc in portList:
            tempPort = serialDevice(portDesc, 9600)
            tempPort.sendCmd('V')
            time.sleep(1)
            r = tempPort.getSerOutput()
            print "Got reply: ", r
            if r.startswith("  V"):
                print "Port:", portDesc, "is dispenser."
                self.dispenserPort = tempPort
                try:
                    self.dispenserPort.sendSyncCmd('I')
                except:
                    print 'Port found but failed to send init command.'
                break
            tempPort.sendCmd('\r\n')
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

    def rotate(self, pt):
        print pt
        if (len(pt) != 7):
            print 'Error: incorrect coordinate string or missing rotation parameters. Dropping rotate instruction'
            return
        cmd = "G02 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]} I{0[5]} J{0[6]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell(1)
        self.currentPosition = pt
        return

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

    # Command fly dispenser to dispense a single fly
    def dispenseFly(self):
        self.dispenserPort.sendSyncCmd('F')
        # Get reply (to check whether fly was successfully dispensed or not)
        r = ""
        while r == "":
            r = self.dispenserPort.getSerOutput()
            time.sleep(0.25)
        reply = r.rstrip("\r\n")
        # 1:dispensed   0:no fly detected   2:fly stuck
        if ( reply == "f"):
            return 1
        elif ( reply == "t" ):
            return 0
        else:
            print 'Fly stuck - clean out dispenser.'
            return 2

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

    # Sends email containing images of arenanum arenas
    def notifyUserFail(self, arenanum, mailfrom, attPic=0, qualPic=25, attImg=1, delFiles=1, robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
        gmail_user = robotEMailAccount
        gmail_password = PWrobotEMailAccount
        try:
            server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
        except:
            print 'SMTP went wrong...'
        try:
            server.ehlo()
        except:
            print 'Handshake went wrong...'
        try:
            server.login(gmail_user, gmail_password)
        except:
            print 'Login went wrong...'
        if attImg == 1:
            msg = MIMEMultipart()
            for imgnum in range(len(arenanum)):
                curarenanum = str(arenanum[imgnum])
                imgname = curarenanum + 'errImage.png'
                with open(imgname, 'rb') as fp:
                    img = MIMEImage(fp.read())
                msg.attach(img)
                if delFiles == 1:
                	os.remove(imgname)
            fp.close()
            arenanum = str(arenanum)
            msg.preamble = 'Arena ' + arenanum + ' failed to unload.'
            msg['Subject'] = 'Failure: Arenas ' + arenanum + ' Withdraw'
        if attPic == 1:
            arenanum = str(arenanum)
            self.light(True)
            time.sleep(0.2)
            self.cam.start_live()
            self.cam.snap_image()
            self.cam.save_image(arenanum + 'errImage.png', 1, jpeq_quality=qualPic)
            self.cam.stop_live()
            self.dwell(50)
            self.light(False)
            msg['Subject'] = 'Failure: Arena ' + arenanum + ' Withdraw'
            msg = MIMEMultipart()
            msg.preamble = 'Arena ' + arenanum + ' failed to unload.'
            fp = open(arenanum + 'errImage.png', 'rb')
            img = MIMEImage(fp.read())
            fp.close()
            msg.attach(img)
            if delFiles == 1:
            	os.remove(arenanum + 'errImage.png')
        if attPic == 0 and attImg == 0:
            arenanum = str(arenanum)
            msg['Subject'] = 'Failure: Arena ' + arenanum + ' Withdraw'
        msg['From'] = gmail_user
        msg['To'] = gmail_user
        server.sendmail(gmail_user, mailfrom, msg.as_string())
        server.quit()
        return

    # Captures arena picture at location (Images named consecutively if multiple coordinates specified)
    def SaveArenaPic(self, Xcoords, Ycoords, IndVect, qualPic=25, Zcam=40, ImgName='errImage.png'):
        self.light(True)
        self.cam.start_live()
        for ImgNum in range(len(Xcoords)):
            self.moveToSpd(pt=[float(Xcoords[ImgNum]), float(Ycoords[ImgNum]), 0, Zcam, 10, 5000])
            self.dwell(50)		# Put higher to reduce effect of motion-caused rig trembling on picture
            self.cam.snap_image()
            curInd = str(IndVect[ImgNum])
            self.cam.save_image(curInd + 'errImage.png', 1, jpeq_quality=qualPic)
            self.dwell(10)
        self.cam.stop_live()
        self.light(False)

    # Reads the most recent emails at associated gmail account and parses instructions (Incorrect instruction format will not cause error-state). Note: formatted for gmail and iOS Mail application.
    def receiveMailInstruct(self, delMail=1, subjKeyVect=['INSTRUCT', 'A2H', 'H2A', 'SWP', 'SWPFL', 'HELP', 'CLCT'], robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):		# Remeber to add all programmed keywords!
	    try:
	        pop_conn = poplib.POP3_SSL('pop.gmail.com')
	        pop_conn.user(robotEMailAccount)
	        pop_conn.pass_(PWrobotEMailAccount)
	        messages = [pop_conn.retr(i) for i in reversed(range(1, len(pop_conn.list()[1]) + 1))]
	        messages = ["\n".join(mssg[1]) for mssg in messages]
	        messages = [parser.Parser().parsestr(mssg) for mssg in messages]
	        for i, message in enumerate(messages):
	            if delMail==1:
	                pop_conn.dele(i+1)
	            if message['subject'] == 'HELP':
	            	return {'values': range(0,2), 'instruct': message['subject'], 'from': message['from']}
	            if message['subject'] in subjKeyVect:
	                print 'Instruction keyword found.'
	                message.set_type('text/plain')
	                try:		# works on gmail account from PC
	                    prefilter = str(message.get_payload(0))
	                    prefilter = prefilter.split('\n')[3].split(',')
	                except:		# works from iphone
	                	prefilter = str(message).split('"us-ascii"')[1].split('\n')[2].split(',')
	                postfilter = range(len(prefilter))
	                for i in range(len(prefilter)):
	                	postfilter[i] = int(prefilter[i])
	                pop_conn.quit()
	                return {'values': postfilter, 'instruct': message['subject'], 'from': message['from']}
	        pop_conn.quit()
	        return
	    except:
	        return None

	# Puts robot in listen mode (Repeated receiveMailInstruct) and updates listening state in online monitor
    def listenMode(self, duration=60, listenInterval=10):		# in seconds
        print 'Listening for', duration, 'seconds in', listenInterval, 'second intervals.'
        # Puts monitor to mode 2 for listening
        try:
            urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=2')
        except:
            print 'Could not reach monitor URL'
        for interval in range(duration/listenInterval):
            time.sleep(listenInterval)
            mail = self.receiveMailInstruct()
            if mail != None:
                return mail
        print 'No instructions received.'
        # Sends 0 to monitor after listening mode is over
        try:
            urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=0')
        except:
            print 'Could not reach monitor URL.'
        return mail

    # Translates correct email commands into preprogrammed robot routines (Incorrect values will cause error-state requiring manual robot reset)
    def doInstruct(self, mailfrom, instruction, values, CamX, CamY, CamZ, ManipX, ManipY, arenaRad, HomeX, HomeY, HomeZwd, HomeZdp, turnZ, vacZ, dispX=639.5, dispY=113, dispZ=34, disponlyifsure=0, maxconsecstuck=6,robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
        if instruction == 'SWP':
            flyremainvect = self.sweep(CamX[values[0]:values[1]], CamY[values[0]:values[1]], camz=CamZ)
            unsurevect = self.sweep(CamX[flyremainvect], CamY[flyremainvect], camz=CamZ)
            try:
                flyremainvect = flyremainvect[unsurevect]
                self.SaveArenaPic(Xcoords=CamX[flyremainvect], Ycoords=CamY[flyremainvect], IndVect=flyremainvect)
                self.notifyUserFail(flyremainvect, mailfrom=mailfrom, attImg=1)
            except:
                print 'No failed arenas detected'
        elif instruction == 'SWPFL':
            self.SaveArenaPic(Xcoords=CamX[values[0]:values[1]], Ycoords=CamY[values[0]:values[1]], IndVect=range(values[0], values[1]))
            self.notifyUserFail(range(values[0], values[1]), mailfrom=mailfrom, attImg=1)
        elif instruction == 'H2A':
            try:		# If not specified, add repetitions as 1
                values[4]
            except:
                values.append(1)
            for repeat in range(0, values[4]):
                for depositValue in range(values[0], values[1]):
                    self.homeWithdraw(homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=HomeZwd)
                    checkmiss = self.arenaDeposit(camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, airPos=values[2], airZ=vacZ, closePos=values[3])
                    if (values[1] - depositValue)%4 == 0:
                        print 'Resetting Z position to maintain accuracy.'
                        self.homeZ()
                    if checkmiss['missonce'] != 0:
                        print 'Missed opening at least once - realigning...'
                        self.homeZ()
        elif instruction == 'A2H':
            try:		# If not specified, add repetitions as 1
                values[5]
            except:
                values.append(1)
            for repeat in range(0, values[5]):
                for depositValue in range(values[0], values[1]):
                    checkmiss = self.arenaWithdraw(camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, vacPos=values[2], vacZ=vacZ, closePos=values[3], vacstrategy=values[4])
                    self.homeDeposit(homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=HomeZdp)
                    if (values[1] - depositValue)%4 == 0:
                        print 'Resetting Z position to maintain accuracy.'
                        self.homeZ()
                    if checkmiss['missonce'] != 0:
                        print 'Missed opening at least once - realigning...'
                        self.homeZ()
        elif instruction == 'HELP':
            gmail_user = robotEMailAccount
            gmail_password = PWrobotEMailAccount
            try:
               server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
            except:
                print 'SMTP went wrong...'
            try:
                server.ehlo()
            except:
                print 'Handshake went wrong...'
            try:
                server.login(gmail_user, gmail_password)
            except:
                print 'Login went wrong...'
            msg = MIMEText('Format: Use KEYWORD in subject line. Enter arenas and settings as "#,#,#,..." without spaces.\n \nKEYWORDS: \nSWP: Sweeps from arenas [1] through [2] and sends back pictures of arenas in which motion was detected.\nSWPFL: Sends back pictures of arenas [1] through [2].\nA2H: Moves flies in arenas [1] through [2] to their homeplates. Vacuums at radian [3] and closes the lid at radian [4] using strategy [5].\nH2A: Same as A2H but from homeplates to arenas.\nCLCT: Starts virgin collection process for [1] seconds total every [2] seconds.\n \nExample:\nSubject: A2H\nText: 0,18,50,180,2.')
            msg['Subject'] = 'RE: SantaFe Remote Access Help'
            msg['From'] = robotEMailAccount
            msg['To'] = mailfrom
            server.sendmail(robotEMailAccount, mailfrom, msg.as_string())
            server.quit()
        elif instruction == 'CLCT':
            self.collectHatchedForT(homecoordX=HomeX, homecoordY=HomeY, dispX=dispX, dispY=dispY, dispZ=dispZ, onlyifsure=disponlyifsure, carefulZ=9, vacBurst=2, homeZ=44, dispiter=1, carryovernDispensed=0, maxconsecstuck=maxconsecstuck, collectT=values[0], collectInt=values[1])


    # Visual representation according to workspace and module dimensions (Hardcoded due to maximum workspace length used in current experiment)
    def visualizeInstruct(self, HXcoords, HYcoords, Xcoords, Ycoords, POIrad, dispx, dispy, instN, instHN=0, instdispN=0, start='home', ArenaSide='L'):
		height = max(HYcoords)*2
		width = max(HXcoords)*2
		background = np.zeros((height,width,3), np.uint8)
		background[:,:] = (170, 170, 170)
		# Robot home
		cv2.rectangle(background, (0, 0), (50, 50), (125, 125, 125), -1)
		# Tray outline
		cv2.rectangle(background, (min(Xcoords)-POIrad*1.5, min(Ycoords)-POIrad*1.5), (max(Xcoords)+POIrad*1.5, max(Ycoords)+POIrad*1.5), (0, 0, 0), -1)
		# Arenas
		for ncirc in range(len(Xcoords)):
		    cv2.circle(background, (Xcoords[ncirc], Ycoords[ncirc]), int(POIrad), (255, 255, 255), 1)
		    cv2.line(background, (Xcoords[ncirc], Ycoords[ncirc]-POIrad*0.75), (Xcoords[ncirc], Ycoords[ncirc]+POIrad*0.75), (255,255,255), thickness=1, lineType=1, shift=0)
		# Fly home outline
		cv2.rectangle(background, (min(HXcoords)-POIrad/2*1.5, min(HYcoords)-POIrad/2*1.5), (max(HXcoords)+POIrad/2*1.5, max(HYcoords)+POIrad/2*1.5), (55, 55, 55), -1)
		# Fly homes
		for ncirc in range(len(HXcoords)):
		    cv2.circle(background, (HXcoords[ncirc], HYcoords[ncirc]), int(POIrad)/3, (255, 255, 255), 1)
		# Dispenser
		cv2.circle(background, (int(dispx), int(dispy)), int(POIrad), (155, 155, 155), -1)
		cv2.circle(background, (int(dispx), int(dispy)), int(POIrad)/2, (0, 0, 0), 2)

		# Visualize instruction (flip text before entire image is flipped for easier mapping)
		mask = 	np.zeros((height,width,3), np.uint8)
		if start == 'arena':		# Considers fly in arena the 'actual' ID
			for ncirc in range(len(instN)):
				if instN[ncirc] <= 9:
					if ArenaSide =='L':
						cv2.putText(mask, str(instN[ncirc]), ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
					elif ArenaSide =='R':
						cv2.putText(mask, str(instN[ncirc]), ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
				else:
					if ArenaSide =='L':
						cv2.putText(mask, str(instN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
						cv2.putText(mask, str(instN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
					elif ArenaSide =='R':
						cv2.putText(mask, str(instN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
						cv2.putText(mask, str(instN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
		elif start == 'home':		# Considers fly in home to be the start ID
			for ncirc in range(len(instHN)):
				if instHN[ncirc] <= 9:
					if ArenaSide =='L':
						cv2.putText(mask, str(instHN[ncirc]), ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
					elif ArenaSide =='R':
						cv2.putText(mask, str(instHN[ncirc]), ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
				else:
					if ArenaSide =='L':
						cv2.putText(mask, str(instHN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
						cv2.putText(mask, str(instHN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
					elif ArenaSide == 'R':
						cv2.putText(mask, str(instHN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
						cv2.putText(mask, str(instHN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
		# Flip text so it lines up with prior image
		mask = cv2.flip(mask,1)
		background = background + mask
		if instdispN != 0:
			mask = 	np.zeros((height,width,3), np.uint8)
			cv2.circle(mask, (int(dispx), int(dispy)), int(POIrad)/2, (0, 255, 255), 2)
			background = background + mask
		if instHN != 0:
			mask = 	np.zeros((height,width,3), np.uint8)
			# Indicate which home plates will be targeted
			for ncirc in range(len(instHN)):
				cv2.circle(mask, (width-HXcoords[instHN[ncirc]], HYcoords[instHN[ncirc]]), int(POIrad)/3, (255, 0, 0), -1)
			# Add a zoomed in home plate for better visibility
			curcoord = range(2)
			hcoordsX = np.zeros((12, 8))
			hcoordsY = np.zeros((12, 8))
			for hrow in range(0,12):
				for hcol in range(0,8):
					curcoord[0] = 350 - (hcol * 18)
					curcoord[1] = float(425) - float(hrow * 18)
					hcoordsX[hrow, hcol] = curcoord[0]
					hcoordsY[hrow, hcol] = float(curcoord[1])
			zoomHX = np.reshape(hcoordsX, (12*8,1))
			zoomHY = np.reshape(hcoordsY, (12*8,1))
			cv2.rectangle(mask, (max(zoomHX)+15, max(zoomHY)+15), (min(zoomHX)-15,min(zoomHY)-15), (115, 115, 115), -1)
			# Also with bigger individual homes
			for ncirc in range(len(HXcoords)):
				cv2.circle(mask, (zoomHX[ncirc], zoomHY[ncirc]), int(POIrad/1.2), (-55, -55, -55), 1)
			if start == 'home':
				for ncirc in range(len(instHN)):
					# Put string of fly in vector instN -> In the future: add left/right & homeToArn or the other way around
					# Larger numbers have to be smaller
					if instHN[ncirc] <= 9:
						cv2.putText(mask, str(instHN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(255,0,0))
					else:
						cv2.putText(mask, str(instHN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(255,0,0))
			elif start =='arena':
				for ncirc in range(len(instN)):
					if instHN[ncirc] <= 9:
						cv2.putText(mask, str(instN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(255,0,0))
					else:
						cv2.putText(mask, str(instN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(255,0,0))
			# Zoom-in lines
			cv2.line(mask, (max(zoomHX)+17, max(zoomHY)+10), (max(HXcoords)-10, max(HYcoords)+10), (35,35,35), thickness=1, lineType=1, shift=0)
			cv2.line(mask, (max(zoomHX)+17, max(zoomHY)-215), (max(HXcoords)-10, max(HYcoords)-100), (35,35,35), thickness=1, lineType=1, shift=0)
			mask = cv2.flip(mask,1)
			background = cv2.subtract(background, mask)
		# Flip entire image vertically so it lines up with reality
		background = cv2.flip(background,1)
		# Robot home text after flip
		cv2.putText(background, 'Robot', (width-43, 20),fontFace=1, fontScale=0.7, color=(0,0,0))
		cv2.putText(background, 'Home', (width-43, 30),fontFace=1, fontScale=0.7, color=(0,0,0))
		key = 0
		while key == 0:
			print 'Press ENTER or SPACE to accept.\nPress ESC to cancel instruction.'
			cv2.imshow("Instruction", background)
			key = cv2.waitKey(0)
			if key != 27 and key != 13 and key != 32:
				key = 0
			elif key == 27:
				cv2.destroyAllWindows()
				quit()
				return False
			elif key == 13 or 32:
				print 'Instruction accepted.'
				cv2.destroyAllWindows()
				return background

	# Visual update of current command
    def updateCurInstruct(self, background, N, HXcoords, HYcoords, Xcoords, Ycoords, dispx, dispy, instN, ArenaSide='L', instHN=0, instdispN=0, start='home'):
		update = background.copy()
		update = cv2.flip(update,1)
		if start =='home':
			if ArenaSide == 'L':
				p = (HXcoords[instHN[N]], HYcoords[instHN[N]])
				q = (Xcoords[instN[N]]+11, Ycoords[instN[N]])
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				angle = np.arctan2(p[1]-q[1], p[0]-q[0])
				p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
				int(q[1] + 9 * np.sin(angle + np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
				int(q[1] + 9 * np.sin(angle - np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
			elif ArenaSide == 'R':
				p = (HXcoords[instHN[N]], HYcoords[instHN[N]])
				q = (Xcoords[instN[N]]+1, Ycoords[instN[N]])
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				angle = np.arctan2(p[1]-q[1], p[0]-q[0])
				p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
				int(q[1] + 9 * np.sin(angle + np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
				int(q[1] + 9 * np.sin(angle - np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
		elif start =='arena':
			if ArenaSide == 'L':
				p = (Xcoords[instN[N]]+11, Ycoords[instN[N]])
				q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				angle = np.arctan2(p[1]-q[1], p[0]-q[0])
				p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
				int(q[1] + 9 * np.sin(angle + np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
				int(q[1] + 9 * np.sin(angle - np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
			elif ArenaSide == 'R':
				p = (Xcoords[instN[N]]+1, Ycoords[instN[N]])
				q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				angle = np.arctan2(p[1]-q[1], p[0]-q[0])
				p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
				int(q[1] + 9 * np.sin(angle + np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
				p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
				int(q[1] + 9 * np.sin(angle - np.pi/4)))
				cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
		elif start == 'dispenser':
			p = (int(dispx), int(dispy))
			q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
			cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
			angle = np.arctan2(p[1]-q[1], p[0]-q[0])
			p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
			int(q[1] + 9 * np.sin(angle + np.pi/4)))
			cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
			p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
			int(q[1] + 9 * np.sin(angle - np.pi/4)))
			cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
		update = cv2.flip(update,1)
		cv2.imshow('Current instruction', update)
		cv2.waitKey(1)
		return {'background':background, 'update':update}

	# Permanently alters the background image and populates arena/home with successes
    def updatePastInstruct(self, background, N, HXcoords, HYcoords, Xcoords, Ycoords, POIrad, dispx, dispy, instN, Fail=0, instHN=0, instdispN=0, ArenaSide='L', start='home', updateWhich='home'):
		curcoord = range(2)
		hcoordsX = np.zeros((12, 8))
		hcoordsY = np.zeros((12, 8))
		for hrow in range(0,12):
			for hcol in range(0,8):
				curcoord[0] = 350 - (hcol * 18)
				curcoord[1] = float(425) - float(hrow * 18)
				hcoordsX[hrow, hcol] = curcoord[0]
				hcoordsY[hrow, hcol] = float(curcoord[1])
		zoomHX = np.reshape(hcoordsX, (12*8,1))
		zoomHY = np.reshape(hcoordsY, (12*8,1))
		width = max(HXcoords)*2
		background = cv2.flip(background,1)
		if updateWhich =='home':
			cv2.circle(background, (HXcoords[instHN[N]], HYcoords[instHN[N]]), int(POIrad)/3, (0,255-Fail*255,255*Fail), -1)
			# zoomed in versions
			if instHN[N] <= 9 and instdispN == 0:
				background = cv2.flip(background,1)
				cv2.putText(background, str(instHN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
				background = cv2.flip(background,1)
			elif instHN[N] > 9 and instdispN == 0:
				background = cv2.flip(background,1)
				cv2.putText(background, str(instHN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(0,255-Fail*255,255*Fail))
				background = cv2.flip(background,1)
			elif instdispN != 0:
				background = cv2.flip(background,1)
				cv2.putText(background, str(instdispN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
				background = cv2.flip(background,1)
		elif updateWhich =='arena':
			if start == 'home':
				background = cv2.flip(background,1)
				if instHN[N] <= 9:
					if ArenaSide == 'L':
						cv2.putText(background, str(instHN[N]), (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
					elif ArenaSide =='R':
						cv2.putText(background, str(instHN[N]), (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
				else:
					if ArenaSide == 'L':
						cv2.putText(background, str(instHN[N])[0], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
						cv2.putText(background, str(instHN[N])[1], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
					elif ArenaSide =='R':
						cv2.putText(background, str(instHN[N])[0], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
						cv2.putText(background, str(instHN[N])[1], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
			elif start == 'arena':
				background = cv2.flip(background,1)
				if instN[N] <= 9:
					if ArenaSide == 'L':
						cv2.putText(background, str(instN[N]), (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
					elif ArenaSide =='R':
						cv2.putText(background, str(instN[N]), (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
				else:
					if ArenaSide == 'L':
						cv2.putText(background, str(instN[N])[0], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
						cv2.putText(background, str(instN[N])[1], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
					elif ArenaSide =='R':
						cv2.putText(background, str(instN[N])[0], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
						cv2.putText(background, str(instN[N])[1], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
			else:
				background = cv2.flip(background,1)
			background = cv2.flip(background,1)
		background = cv2.flip(background,1)
		cv2.destroyAllWindows()
		cv2.imshow('Current instruction', background)
		cv2.waitKey(1)
		return background

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
		self.cam.start_live()
		self.cam.snap_image()
		self.cam.save_image(''.join(['curImage1.jpg']), 1, jpeq_quality=100)
		self.cam.stop_live()
		self.dwell(800)
		self.cam.start_live()
		self.cam.snap_image()
		self.cam.save_image(''.join(['curImage2.jpg']), 1, jpeq_quality=100)
		self.cam.stop_live()
		self.light(False)
		image1 = cv2.imread('curImage1.jpg')
		image2 = cv2.imread('curImage2.jpg')
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


def availablePorts():
    """Lists serial ports"""

    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def iround(x):
    y = round(x) - .5
    return int(y) + (y > 0)
