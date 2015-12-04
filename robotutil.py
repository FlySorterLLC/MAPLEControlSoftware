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
import glob
import serial
import time
import ConfigParser

import pyicic.IC_ImagingControl

# Serial communications class that is used for multiple devices.
# In Santa Fe, the printrboard and the Z axis motor controllers
# are each serial devices. The printrboard is connected directly
# via USB, and the Z axis controllers (Synaptron boards from Solutions Cubed)
# are connected together on an RS232 serial bus, which is connected to
# the computer by a USB-to-Serial converter.
#
# 1. Printrboard - controls X, Y, and rotation for Z0 and Z2
# 2. Motor control board (3x Synaptron boards + USB-to-serial converter) - up & down on Z0, Z1 and Z2
#
# So this class will be instantiated twice (at least) -- once for the
# printrboard, and once for the Z axis controllers (one serial port for
# all three controllers).
#
# Documentation is available online for G-codes (printrboard) and the Synaptron controller:
#
# http://reprap.org/wiki/G-code
# http://solutions-cubed.com/app-notes-downloads/#SYNAPU
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

    printrboardPort = ""
    synaptronsPort = ""
    dispenserPort = ""

    # These variables should be kept up-to-date by functions that change them
    currentPosition = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    currentRotation = np.array([0.0, 0.0])

    isInitialized = False

    # There are 6 encoder counts per revolution, gear ratio of motor is 29.86:1,
    # leadscrew pitch is 1/10.4" and there are 25.4 mm/in
    ZCountsPerMM = 6.0*29.86*10.4/25.4

    # Configuration defaults
    configDefaults = {'WorkspaceXSize': '1000',
                      'WorkspaceYSize': '270',
                      'MaxZ1Depth': '68',
                      'MaxZ2Depth': '50',
                      'MaxZ3Depth': '55',
                      'ZAddressBase': '54',
                      'OutputDir': 'Photos',
                      }
    
    def __init__(self, robotConfigFile):
        print "Reading config file..."
        self.config = ConfigParser.RawConfigParser(self.configDefaults)
        self.readConfig(robotConfigFile)

        
        print "Initializing serial connections..."
        self.printrboard = None
        self.synaptrons = None

        
        # Search serial ports, look for motor control board and printrboard
        portList = availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            # print "Trying port", portDesc
            tempPort = santaFeSerial(portDesc)
            if tempPort.sendCmdGetReply("M115\r\n").startswith("FIRMWARE_NAME"):
                print "Port:", portDesc, "is printrboard."
                self.printrboardPort = portDesc
                self.printrboard = tempPort
                continue
            if tempPort.sendCmdGetReply("54,00,\r\n").startswith("54"):
                print "Port:", portDesc, "is motor controller."
                self.synaptronsPort = portDesc
                self.synaptrons = tempPort
                continue
            tempPort.close()

        if self.printrboard is None or self.synaptrons is None:
            print "Serial initialization failed."
            if self.printrboard is None:
                print "Printrboard not found."
            else:
                self.printrboard.close()
            if self.synaptrons is None:
                print "Synaptrons not found."
            else:
                self.synaptrons.close()
            return

        self.cam = cameraInit()
        if self.cam == None:
            print "Camera init fail."
            self.synaptrons.close()
            self.printrboard.close()
            return
       
        self.currentPosition = self.getCurrentPosition()
        self.isInitialized = True
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
        self.WorkspaceSize = np.array( [ float(self.config.get('DEFAULT', 'WorkspaceXSize')), float(self.config.get('DEFAULT', 'WorkspaceYSize')) ] )
        self.ZAddressBase = int(self.config.get('DEFAULT', 'ZAddressBase'))
##        self.MaxZDepth
##                      'MaxZ1Depth': '50',
##                      'MaxZ2Depth': '60',
##                      'MaxZ3Depth': '40',
##                      'ZBaseAddress': '54',
        return
        
    # Write out the config file after transferring values from vars
    def writeConfig(self, configFile):
##        self.config.set('DEFAULT', 'CamIndex', CamIndex)
##        self.config.set('DEFAULT', 'OutputDir', OutputDir)
##        with open('FlySorter.cfg', 'wb') as configfile:
##            self.config.write(configfile)
        return
        
    def release(self):
        self.light(False)
        self.flyManipVenturi(False)
        self.smallPartManipVenturi(False)
        self.flyManipAir(False)
        self.smallPartManipAir(False)
        self.cam.close()
        self.printrboard.close()
        self.synaptrons.close()

    def home(self):
        # First home 3 small motors for z axes
        self.homeZAxis(0)
        self.homeZAxis(1)
        self.homeZAxis(2)
        # Then home Y, then X (in that order, b/c of CoreXY configuration)
        self.printrboard.sendSyncCmd("G28 Y\n")
        self.printrboard.sendSyncCmd("G28 X\n")
        self.printrboard.sendSyncCmd("G01 F10000\n")
        self.currentPosition = np.array([0., 0., 0., 0., 0.])
        return
        
    def homeZAxis(self, zNum):
        # To home the Z axes, move it all the way up,
        # wait for the limit switch to be engaged,
        # disable motor, reset position and setpoint,
        # re-enable motor
        print "Homing Z axis", zNum
        # start axis moving up
        address = self.ZAddressBase+zNum
        cmd = "{0},39,-7000\r\n".format(address)
        self.synaptrons.sendSyncCmd(cmd)
        # keep moving up until limit switch engaged
        zHomedBool=0
        while (zHomedBool==0):
            zStatus = self.synaptrons.sendCmdGetReply(str(address)+",04,\r\n")
            zHomedBool = int(zStatus.split(',')[1])
            zHomedBool = '{0:016b}'.format(zHomedBool)
            # print 'z-axis homed?', zHomedBool[13]
            zHomedBool=int(zHomedBool[13])

        # Temporarily set Z position to zero
        self.synaptrons.sendSyncCmd("{0},03,22\r\n".format(address)) # Stop motor
        self.synaptrons.sendSyncCmd("{0},05,0\r\n".format(address)) # Set position to zero
        self.synaptrons.sendSyncCmd("{0},39,0\r\n".format(address)) # Set set point to zero
        self.synaptrons.sendSyncCmd("{0},03,6\r\n".format(address)) # Re-enable motor

        # Now move it back down until limit switch is disengaged
        zHomedBool = 1
        zPos = 15 # we know there are at least 15 encoder steps before the switch is disengaged
        while (zHomedBool == 1):
            cmd = "{0},39,{1}\r\n".format(address, zPos)
            self.synaptrons.sendSyncCmd(cmd)
            zStatus = self.synaptrons.sendCmdGetReply(str(address)+",04,\r\n")
            zHomedBool = int(zStatus.split(',')[1])
            zHomedBool = '{0:016b}'.format(zHomedBool)
            # print 'zPos', zPos, 'z-axis homed?', zHomedBool[13]
            zHomedBool=int(zHomedBool[13])
            zPos=zPos+1

        # Once again, set position to zero
        self.synaptrons.sendSyncCmd("{0},03,22\r\n".format(address)) # Stop motor
        self.synaptrons.sendSyncCmd("{0},05,0\r\n".format(address)) # Set position to zero
        self.synaptrons.sendSyncCmd("{0},39,0\r\n".format(address)) # Set set point to zero
        self.synaptrons.sendSyncCmd("{0},03,6\r\n".format(address)) # Re-enable motor
        
        self.currentPosition[2+zNum] = 0.
        return

    def getZPos(self, zNum):
        address = self.ZAddressBase+zNum
        zPosStr = self.synaptrons.sendCmdGetReply(str(address)+",05,\r\n")
        zPos = float(zPosStr.split(',')[1])/self.ZCountsPerMM
        #print zPos
        return zPos
    
    def getCurrentPosition(self):
        positions = self.printrboard.sendCmdGetReply("M114\n").split(' ')
        # M114 returns string like: "X:10.00 Y:10.00 Z:0.00 E:0.00 Count X: 19.99 Y:0.00 Z:0.00"
        xPos = float(positions[0].split(':')[1])
        yPos = float(positions[1].split(':')[1])
        return np.array( [ xPos, yPos, self.getZPos(0), self.getZPos(1), self.getZPos(2) ] )

    def moveXY(self, pt):
        if (len(pt) != 2):
            print 'Error: incorrect coordinate string. Dropping moveXY instruction'
            return

        #start x and y on their way
        cmd = "G01 X{0[0]} Y{0[1]}\n".format(pt)
        self.printrboard.sendSyncCmd(cmd)
        self.currentPosition[0] = pt[0]
        self.currentPosition[1] = pt[1]
        

    def moveZ(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string. Dropping moveZ instruction'
            return
        
        # set z precision in mm
        zErrorBand = 0.1

        baseZAddr=self.ZAddressBase
#        self.synaptrons = santaFeSerial(self.synaptronsPort)
        self.synaptrons.sendSyncCmd(str(baseZAddr+0)+",39,{0}\r\n".format(int(pt[2]*self.ZCountsPerMM)))
        self.synaptrons.sendSyncCmd(str(baseZAddr+1)+",39,{0}\r\n".format(int(pt[3]*self.ZCountsPerMM)))
        self.synaptrons.sendSyncCmd(str(baseZAddr+2)+",39,{0}\r\n".format(int(pt[4]*self.ZCountsPerMM)))

        zBoolArray = [0, 0, 0, 0, 0]
        while (sum(zBoolArray) < 3):   
            #check to see if all z-axes have arrived
            for i in range(0,3):
                zTemp = self.synaptrons.sendCmdGetReply(str(baseZAddr+i)+",05,\r\n")
                zTemp = int(zTemp.split(',')[1])/self.ZCountsPerMM
                zBoolArray[2+i] = int(abs(zTemp - pt[2+i]) < zErrorBand)
                #print 'i is: ', i,' zstring is: ', zTemp, ' zBoolArray is: ', zBoolArray
  
        print 'Moved to Z coordinates ', pt
        self.currentPosition[2] = pt[0]
        self.currentPosition[3] = pt[1]
        self.currentPosition[4] = pt[2]
        return
        
        
    def moveTo(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string. Dropping moveTo instruction'
            return
        
        # set z precision in mm
        zErrorBand = 0.1

        #start z-axes on their way
        baseZAddr=self.ZAddressBase
#        self.synaptrons = santaFeSerial(self.synaptronsPort)
        self.synaptrons.sendSyncCmd(str(baseZAddr+0)+",39,{0}\r\n".format(int(pt[2]*self.ZCountsPerMM)))
        self.synaptrons.sendSyncCmd(str(baseZAddr+1)+",39,{0}\r\n".format(int(pt[3]*self.ZCountsPerMM)))
        self.synaptrons.sendSyncCmd(str(baseZAddr+2)+",39,{0}\r\n".format(int(pt[4]*self.ZCountsPerMM)))

        #start x and y on their way
        self.moveXY( (pt[0], pt[1]) )
        self.dwell(1)
        
        zBoolArray = [0, 0, 0, 0, 0]
        while (sum(zBoolArray) < 3):   
            #check to see if all z-axes have arrived
            for i in range(0,3):
                zTemp = self.synaptrons.sendCmdGetReply(str(baseZAddr+i)+",05,\r\n")
                zTemp = int(zTemp.split(',')[1])/self.ZCountsPerMM
                zBoolArray[2+i] = int(abs(zTemp - pt[2+i]) < zErrorBand)
                #print 'i is: ', i,' zstring is: ', zTemp, ' zBoolArray is: ', zBoolArray
  
        print 'Moved to coordinates ', pt
        self.currentPosition[2] = pt[2]
        self.currentPosition[3] = pt[3]
        self.currentPosition[4] = pt[4]
        return

    def moveRel(self, pt):
        self.moveTo(self.currentPosition + pt)

    def moveXYList(self, ptList):
        # TODO: check shape of list
        for pt in ptList:
            moveXY(pt)
        return

    def dwell(self, t):
        cmd = "G04 P{0}\n".format(t)
        self.printrboard.sendSyncCmd(cmd)
        return

    def light(self, onOff = False):
        if ( onOff == True ):
            cmd = "M106\n"
        else:
            cmd = "M107\n"
        self.printrboard.sendCmd(cmd)
        return

    def flyManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P9 S255\n"
        else:
            cmd = "M42 P9 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def smallPartManipAir(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P10 S255\n"
        else:
            cmd = "M42 P10 S0\n"
        self.printrboard.sendCmd(cmd)
        return
    
    def flyManipVenturi(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P12 S255\n"
        else:
            cmd = "M42 P12 S0\n"
        self.printrboard.sendCmd(cmd)
        return
    
    def smallPartManipVenturi(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P11 S255\n"
        else:
            cmd = "M42 P11 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def spinSmallPartManip(self, angle):
        cmd = "G01 E{0:.4}\n".format(float(angle))
        self.printrboard.sendCmd(cmd)
        return

    def spinFlyManip(self, angle):
        cmd = "G01 Z{0:.4}\n".format(float(angle))
        self.printrboard.sendCmd(cmd)
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

