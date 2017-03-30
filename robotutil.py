## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: robotutil.py
#  Description: Contains classes and functions used to control
#     the FlySorter automated experiment platform (project name Santa Fe).
#

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

import pyicic.IC_ImagingControl

# Serial communications class that is used for multiple devices.
# In Santa Fe, the smoothie board is a serial devices. The smoothie is connected directly
# via USB.
#
# 1. Smoothieboard - controls X, Y, Z0, Z1 and Z2 (X, Y, Z, A and B axes, respectively)
#
# Documentation is available online for G-codes (smoothie):
#
# http://smoothieware.org/supported-g-codes and
# http://reprap.org/wiki/G-code
#
class santaFeSerial:
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
        #print "WFO Output:", output
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
            tempPort = santaFeSerial(portDesc, 115200)
            if tempPort.sendCmdGetReply("version\n").startswith("Build version"):
                print "Port:", portDesc, "is smoothie."
                self.smoothiePort = portDesc
                self.smoothie = tempPort
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
        return

    def captureImage(self):
        self.cam.start_live()
        self.cam.snap_image()
        (imgdata, w, h, d) = self.cam.get_image_data()
        self.cam.stop_live()

        #print "Image is", w, "by", h

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

    def release(self):
        self.light(False)
        self.flyManipVac(False)
        self.smallPartManipVac(False)
        self.flyManipAir(False)
        self.smallPartManipAir(False)
        self.cam.close()
        self.smoothie.close()

    def home(self):
        self.smoothie.sendSyncCmd("G28\n")
        self.smoothie.sendSyncCmd("G01 F{0}\n".format(self.travelSpeed))
        self.currentPosition = np.array([0., 0., 0., 0., 0.])
        return

    def getCurrentPosition(self):
        # M114.2 returns string like: "ok MCS: X:0.0000 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000"
		while True:
			try:
				positions = self.smoothie.sendCmdGetReply("M114.2\n").split(' ')
				xPos  = float(positions[2].split(':')[1])
				break
			except:
				print 'Error: retrying to get position'
		xPos  = float(positions[2].split(':')[1])
		yPos  = float(positions[3].split(':')[1])
		z0Pos = float(positions[4].split(':')[1])
		z1Pos = float(positions[5].split(':')[1])
		z2Pos = float(positions[6].split(':')[1])
		return np.array( [ xPos, yPos, z0Pos, z1Pos, z2Pos ] )

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
    
    def moveCirc(self, r, n, endpos, spd, rest, full=False):        #moveCirc2 works better!
    ## circular motion around current point; r: radius, n: amount of points (fine-ness),
    ## endpos: movement end point in multiples of pi, spd; speed, rest: time between movements (fine-ness)
        self.dwell(t=500) # to get accurate starting point
        curcircle = np.arange(n*2).reshape((n, 2))
        XYPos = self.getCurrentPosition()
        CircStart = XYPos[0:2]   # may be interesting in other functions
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
        self.dwell(t=200)   # ensures plate is still
        return CircEnd

    def moveCirc2(self, mid, r, n=360, startpos=0, endpos=360, spd=1500, rest=100, z=53, full=True, retreatZ=10, descendZ=9):       #also lowers onto z height
        careful = 0     # failsafe when already starts lowered
        if spd >= 2001:
            print 'adjusting speed to safe level 2000 ...'	# locks up on cross-axis motion and screws up positioning	
            spd = 2000
        while startpos > 360:
            startpos = startpos - 360
            print 'startpos corrected to:', startpos
        while endpos > 360:
            endpos = endpos - 360
            print 'endpos corrected to:', endpos
        deg = np.ones((n+1,3))
        for x in range(0,len(deg)):
            deg[x,:] = (math.sin(2*math.pi/n*x)*r)+mid[0], (math.cos(2*math.pi/n*x)*r)+mid[1], spd
            #print 'calculation for degree', x, ': ', deg[x,:]
        #print 'calculated points as', deg
        if endpos - startpos < 0:
            step = -1
            offset = -2
        else:
            step = 1
            offset = 2
        #print 'made it to loop with step:', step, 'and offset:', offset
        for i in range(startpos, endpos+1, step):       # loop that performs the circular movement and lowering only on iteration 1
            if careful != 1:
                if step == 1 and i >= endpos-offset:
                    deg[:,2] = self.travelSpeed     # resets slow speed to config normal
                    #print 'speed reset to:', self.travelSpeed   # Has to be before move command to remain in effect
                elif step == -1 and i <= endpos-offset:
                    deg[:,2] = self.travelSpeed
                    #print 'speed reset to:', self.travelSpeed
                self.moveXYSpd(pt=deg[i,:])
                self.dwell(rest)
                if i == startpos:
                    self.dwell(10)  # just a tiny buffer before lowering
                    #print 'waiting to lower...'
                XYpos = self.getCurrentPosition()
                #print i, XYpos, deg[i,0], deg[i,1]
                if i == startpos and (XYpos[0] - deg[startpos,0] <= 1) and (XYpos[1] - deg[startpos,1] <= 1):   # only lower on first iteration and if degrees match
                    #print 'lowering...'
                    self.dwell(10)
                    self.moveZ(pt=[XYpos[0],XYpos[1],0,0,z-descendZ]) # lowers to 9 units above detected opening
                    self.dwell(1) # give time before the loop
                    #print self.getCurrentPosition()
                    #self.moveTo(pt=[XYpos[0],XYpos[1],0,0,z-9])        # one of these gets skipped for some reason..
                    #self.dwell(1) # give time before the loop
                    #print self.getCurrentPosition()
                    for j in range(1, descendZ+2):     #carefully lower into opening and start at 0 just to also check current limit
                        self.dwell(1)
                        self.moveZ(pt=[XYpos[0],XYpos[1],0,0,(z-descendZ)+j])
                        #print 'lowering step:', j, 'ends at', self.getCurrentPosition()
                        careful = self.getLimit()
                        if careful == 1:
                            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                            #print 'should be retreating...'  
                            #print 'firstbreak' 
                            break
            elif careful != 0:
                self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                #print 'secbreak'
                break      #super overkill but more secure
        if careful != 1 and full == True:
            self.moveXYSpd(pt=deg[endpos,:])
            XYpos = deg[endpos,0:2]
            self.dwell(1)
        elif careful != 0:
            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
        return {'endXY':XYpos, 'endDeg':endpos, 'oldMid': mid, 'limit': careful, 'startDeg': startpos}


    def tryOpening(self, mid, r, n=360, startpos=0, endpos=360, spd=1000, rest=5, z=53, full=True, retreatZ=10, descendZ=9):
        tryspd = spd
        trymid = mid
        trystart = startpos
        tryend = endpos
        tryz = z
        unsure=0
        radi = r
        #print 'first try...'
        trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=trystart, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
        startposFirst = startpos
        #print 'startposFirst is:', startposFirst
        while trylower['limit'] == 1 and unsure != 1:
            for cw in xrange(2,10,2):
                if trylower['limit'] == 1:
                    #print 'trying 2 degrees further clockwise...'
                    startpos = startpos + cw
                    trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            startpos = startposFirst
            #print 'now trying counterclockwise at', startpos, 'which should match', startposFirst
            for ccw in xrange(2,10,2):       # can skip 0 degree offset bc clockwise covered it
                if trylower['limit'] == 1:
                    #print 'trying 2 degrees further counter-clockwise...'
                    startpos = startpos - ccw
                    trylower = self.moveCirc2(mid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            if trylower['limit'] == 1:
                print 'could not find opening - detecting anew...'
                unsure = 1
        return trylower

    def lowerCare(self, z, descendZ=9, retreatZ=18):		# z: how low to move; descendZ: how many steps are done carefully to reach z; retreatZ: RELATIVE height retreat upon hit
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
			#print 'lowering step:', i, 'ends at', self.getCurrentPosition()
			careful = self.getLimit()
			if careful == 1:
				self.moveRel(pt=[0,0,0,0,-retreatZ])
				break
		posend = self.getCurrentPosition
		return {'pos.begin': posbegin, 'pos.end': posend, 'limit': careful}

    def homeWithdraw(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=45):
        if refptX != 'N':
	        self.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
	        self.dwell(t=1)
        self.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, 10, 5000])        # Go to actual home
        self.dwell(t=1)
        self.flyManipAir(True)
        trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
        if trylowerHome['limit'] == 0:
            self.dwell(t=1)
            for b in range(0,vacBurst):
                self.flyManipAir(True)
                self.dwell(t=rand.choice(range(2,4)))
                self.flyManipAir(False)
                self.dwell(t=rand.choice(range(2,4)))
            self.smallPartManipVac(True)
            self.dwell(t=2000)
            self.moveRel(pt=[0, 0, 0, 0, -homeZ])
            self.dwell(t=10)
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

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
        return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

    def getLimit(self):     # if this breaks look at position of limit max B in the string!
        templimit = str(self.smoothie.sendCmdGetReply("M119\n").split(' '))
        #print templimit
        limit = int(templimit[150])
        #print 'templimit is', templimit
        if limit == 1:
            #self.home()
            print 'limit is', limit, '!'
            #self.smoothie.sendCmd("M999\n")
        return limit


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

    def moveRel(self, pt):
        #print "Curr pos:", self.currentPosition
        self.moveTo( map(sum,zip(self.currentPosition, pt)) )

    def moveXYList(self, ptList):
        # TODO: check shape of list
        for pt in ptList:
            self.moveXY(pt)
        return

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

    def dwell(self, t):
		while True:
			try:
				cmd = "G04 P{0}\n".format(t)
				self.smoothie.sendSyncCmd(cmd)
				break
			except:
				print 'Error: retrying to send dwell command'
		return

    def light(self, onOff = False):
        if ( onOff == True ):
            cmd = "M48\n"
        else:
            cmd = "M49\n"
        self.smoothie.sendCmd(cmd)
        return

    def flyManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M46\n"
        else:
            cmd = "M47\n"
        self.smoothie.sendCmd(cmd)
        return

    def flyManipVac(self, onOff = False):		# rerouted pins to smallPart vacuum
        if (onOff == True):
            cmd = "M44\n"
        else:
            cmd = "M45\n"
        self.smoothie.sendCmd(cmd)
        return

    def smallPartManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M42\n"
        else:
            cmd = "M43\n"
        self.smoothie.sendCmd(cmd)
        return

    def smallPartManipVac(self, onOff = False):		# rerouted to fly vacuum
        if (onOff == True):
            cmd = "M40\n"
        else:
            cmd = "M41\n"
        self.smoothie.sendCmd(cmd)
        return

    def flyManipVenturi(self, onOff = False):
        print "WARNING - flyManipVenturi deprecated! Call location:"
        traceback.print_exc(file=sys.stdout)
        return

    def smallPartManipVenturi(self, onOff = False):
        print "WARNING - smallPartManipVenturi deprecated! Call location:"
        traceback.print_exc(file=sys.stdout)
        return

    def puffForTime(self, duration):
        print 'Puffing for', duration, 'seconds'
        self.flyManip(False)
        self.vacuum(False)
        self.air(True)
        self.smallPartManip(True)
        time.sleep(duration)
        self.smallPartManip(False)
        self.air(False)
        return

    def suckForTime(self, duration):
        print 'Sucking for', duration, 'seconds'
        self.air(False)
        self.smallPartManip(False)
        self.flyManip(True)
        self.vacuum(True)
        time.sleep(duration)
        self.flyManip(False)
        self.vacuum(False)
        return

    def dipAndGetFly(self, pt, duration, plateBool):
        print 'Dipping to get fly.'
        pt2=list(pt)
        pt2[4]+=10+plateBool*2

        self.puffForTime(0.2)
        time.sleep(0.05)
        self.puffForTime(0.2)
        time.sleep(0.05)
        self.flyManip(True)
        self.vacuum(True)
        self.moveTo(pt2)
        time.sleep(duration)

        pt2[4]+=-(10+plateBool*2)
        self.moveTo(pt2)
        return

    def depositInMaze(self, pt, duration):
        print 'Dipping to vent fly.'
        pt2=list(pt)
        pt2[4]+=10
        self.moveTo(pt2)
        pt2[1]+=-self.mazeSlideOffset
        self.moveTo(pt2)
        self.puffForTime(duration)
        pt2[1]+=self.mazeSlideOffset
        self.moveTo(pt2)
        pt2[4]+=-10
        self.moveTo(pt2)
        return

    def suckFromMaze(self, pt, duration):
        print 'Dipping to vent fly.'
        pt2=list(pt)
        pt2[4]+=10
        self.moveTo(pt2)
        pt2[1]+=-self.mazeSlideOffset
        self.moveTo(pt2)
        self.suckForTime(duration)
        pt2[1]+=self.mazeSlideOffset
        self.moveTo(pt2)
        pt2[4]+=-10
        self.moveTo(pt2)
        return

    def dipAndDropFly(self, pt, duration):
        print 'Dipping to vent fly.'
        pt2=list(pt)
        pt2[4]+=10
        self.moveTo(pt2)
        self.puffForTime(duration)
        pt2[4]+=-10
        self.moveTo(pt2)
        return

	def findFly(self, image):
        # Cribbed from Will's imgprocess script
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
                # We want transform from pixels to mm, using self.FOV
                # Find the shape of the image
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

    def sweep(self, ptsx, ptsy, camz=40, spd=5000):
		detectvect = range(len(ptsx))
		for i in range(len(ptsx)):
			self.moveToSpd(pt=[float(ptsx[i]), float(ptsy[i]), 0, camz, 0, spd])
			detectvect[i] = self.detectFly()
		return np.array(detectvect, dtype = bool)


	## Slowmode = False can be used as a show mode of where circles were detected.
    def findOpening(self, image, slowmode=False, MAX_SIZE=70, MIN_SIZE=60, startp1=119, startp2=150, startp3=2.7):
        result = []
        #MAX_SIZE = 75  # range of size in pixels of the circle lid hole
        #MIN_SIZE = 60
        #cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        #cv2.imshow("thresh", thresh)
        #cv2.waitKey(0)
        #tartp1 = 119
        #startp2 = 147
        #startp3 = 2.6
        detect = 0
        if slowmode == False:
            image = cv2.imread(image)
            #cv2.imshow("output", image)
            #cv2.waitKey()
            image = cv2.resize(image, (1280, 960))
            #cv2.imshow("resized", image)
            #cv2.waitKey()
            output = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("gray", gray)
            #cv2.waitKey(0)
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            while detect == 0:      # decrease sensitivity until at least one circle is found
                #print 'sensitivity lowered to:', startp2
                circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
                 ## change param 1 and 2 for more-less circles
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
                    elif startp2 > 100 and len(circles) > 1:     #probably not starting to find only one circle
                        startp2 = startp2 - 3
                    elif startp2 <= 100 or len(circles) > 1:
                        detect = 1      # to get out if too many circles get found repeatedly -- unlikely to result in wrong angle as it needs validation
                else:
                    startp2 = startp2 - 3       # get less sensitive if no circles were found
                    #print 'reducing sensitivity to...', startp2
            #cv2.imshow("gray", output)
            #cv2.waitKey(0)
        elif slowmode == True:      # Improves findOpening on the off-chance that something wrong in the image processing
            oldcircles = np.zeros((1,3), dtype=np.int)
            certain = 0
            while certain != 1:
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
                        print 'detected ',len(circles), 'circles'
                        detect = 1
                        if oldcircles[0,2] != 0:
                            if (oldcircles[0,0] - circles[0,0]) <= 30 and (oldcircles[0,1] - circles[0,1]) <= 30 and (oldcircles[0,2] - circles[0,2]) <= 20:
                                certain = 1
                                #print circles, 'found NOT different from', oldcircles
                            else:
                                #print circles, 'found different from', oldcircles
                                oldcircles = circles
                        else:
                            oldcircles = circles
                    else:
                        startp2 = startp2 - 3       # get less sensitive if no circles were found
                        #print 'reducing sensitivity...'
    # show the output image
        #cv2.imshow("output", output)
        #cv2.waitKey(0)
        return circles

    # Returns degrees of a point's coordinates relative to the image midpoint (the opening)
    def getDegs(self, img, img_width=1280, img_height=960):
        imgmid = [img_width/2, img_height/2]
        if len(img[:,1]) >= 2 and not (img[0,0] - img[1,0] >= 150) and not (img[0,1] - img[1,1] >= 150):    # allows 2 close circles and takes mean coords
            img[0,0] = (img[0,0] + img[1,0])/2
            img[0,1] = (img[0,1] + img[1,1])/2
            print 'correcting...'
        dx = img[0,0] - imgmid[0]
        #print 'x difference:', img[0,0], dx
        dy = (img_height - img[0,1]) - imgmid[1]
        #print 'y difference:', img[0,1], dy
        rads = math.atan2(-dy,dx)
        rads %= 2*math.pi
        degs = math.degrees(rads)
        degs = (degs-90)* (-1)
        if degs <= 0:       # ghettomath but works out
            degs = 360 + degs
        if degs <= 180:
            degs = (degs - 180) * (-1)
        elif degs >= 180 and degs <= 360:
            degs = ((degs - 360) * (-1)) + 180
        #print 'detected at', degs, 'degrees'
        return degs

    # Combines findOpening and getDegs
    def findDegs(self, slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=62, startp1=119, startp2=147, startp3=2.6):
    	trymax=MAX_SIZE
    	trymin=MIN_SIZE
    	try1 = startp1
    	try2 = startp2
    	try3 = startp3
    	slwmd = slowmode
        if slowmode == True:
            certain = 0
            while certain != 1:
                tempdeg = np.arange(2)
                #print 'at start tempdeg is', tempdeg
                for i in range(0,2):
                    self.light(True)
                    time.sleep(0.2)
                    self.cam.start_live()
                    self.cam.snap_image()
                    self.cam.save_image(''.join(['curImage.jpg']), 1, jpeq_quality=100)
                    self.cam.stop_live()
                    self.light(False)
                    img = self.findOpening('curImage.jpg', slowmode=slwmd, MAX_SIZE=trymax, MIN_SIZE=trymin, startp1=try1, startp2=try2, startp3=try3)
                    tempdeg[i] = self.getDegs(img)
                #    print 'after loop tempdeg is', tempdeg
                if abs(tempdeg[0] - tempdeg[1]) <= precision:
                    certain = 1
                    #print 'final degree decision is', np.mean(tempdeg)
                    return np.mean(tempdeg)
        elif slowmode == False:
            self.light(True)
            time.sleep(0.2)
            self.cam.start_live()
            self.cam.snap_image()
            self.cam.save_image(''.join(['curImage.jpg']), 1, jpeq_quality=100)
            self.cam.stop_live()
            self.light(False)
            img = self.findOpening('curImage.jpg', slowmode=False)
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
