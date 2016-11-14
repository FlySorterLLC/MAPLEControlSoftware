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

    travelSpeed = 10000
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
        positions = self.smoothie.sendCmdGetReply("M114.2\n").split(' ')
        # M114.2 returns string like: "ok MCS: X:0.0000 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000"
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

    def moveZ(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string. Dropping moveZ instruction'
            return

        if ( self.isPtInBounds(pt) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return

        cmd = "G01 Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[2] = pt[2]
        self.currentPosition[3] = pt[3]
        self.currentPosition[4] = pt[4]
        return


    def moveTo(self, pt):
        if (len(pt) != 5):
            print 'Error: incorrect coordinate string. Dropping moveTo instruction'
            return

        if ( self.isPtInBounds(pt) == False ):
            print 'Error: point out of bounds (less than zero, greater than maxExtents)'
            return

        cmd = "G01 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell(1)

        self.currentPosition = pt
        return

    def moveRel(self, pt):
        #print "Curr pos:", self.currentPosition
        self.moveTo(self.currentPosition + pt)

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
        cmd = "G04 P{0}\n".format(t)
        self.smoothie.sendSyncCmd(cmd)
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

    def flyManipVac(self, onOff = False):
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

    def smallPartManipVac(self, onOff = False):
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
