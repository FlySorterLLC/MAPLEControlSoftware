## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0

import cv2
import numpy as np
import sys
import glob
import serial
import time

class santaFeSerial:
    """Serial class for robot device."""
    
    WaitTimeout = 1
    portName = ""

    def __init__(self, port, baud = 9600, timeout = 0.25):
        self.isOpened = False
        try:
            self.ser = serial.Serial(port, baudrate = baud, timeout = timeout)
        except:
            print "Failed to open port", port
            return
        self.isOpened = True

    def close(self):
        self.ser.close()
        
    def getSerOutput(self):
        #print "GSO:"
        output = ''
        while True:
            # read() blocks for the timeout set below *if* there is nothing to read
            #   otherwise it returns immediately
            byte = self.ser.read(1)
            if byte is None or byte == '':
                break
            output += byte
            if byte == '\n':
                break
        #print "GSO Output:", output
        return output

    def waitForOK(self):
        #print "WFO:"
        output = ''
        timeoutMax = self.WaitTimeout / self.ser.timeout
        timeoutCount = 0
        while True:
            byte = self.ser.read(1)
            if byte is None or byte == '':
                timeoutCount += 1
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
    def sendSyncCmd(self, cmd):
        #print "SSC:", cmd
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        self.waitForOK()

    # Send a command and return the reply
    def sendCmdGetReply(self, cmd):
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        return self.getSerOutput()

    def sendFile(self, filename):
        return 1


class santaFe:
    """Class for fly manipulation robot."""

    # Constants

    
    # There are 6 encoder counts per revolution, gear ratio of motor is 29.86:1,
    # leadscrew pitch is 1/10.4" and there are 25.4 mm/in
    ZCountsPerMM = 6.0*29.86*10.4/25.4 # = 79.36
    ZAddressBase = 54

    printrboardPort = ""
    synaptronsPort = ""

    imageCenter = np.float32( [2592.0/2.0, 1944.0/2.0 ])

    currentPosition = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    currentRotation = np.array([0.0, 0.0])

    isInitialized = False
    
    def __init__(self, camIndex):
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

        # Temporary hack! Synaptrons don't like when you switch
        # between the different addresses, so we open & close the port
        # for each operation.
        self.synaptrons.close()

        self.cam = cameraInit(camIndex)
        if self.cam == None:
            print "Camera init fail."
            self.synaptrons.close()
            self.printrboard.close()
            return

        print "Homing..."
        self.home()
        self.dwell(1)
        self.printrboard.sendSyncCmd("G01 F5000\n")
        
        self.isInitialized = True
        return

    def release(self):
        self.cam.release()
        self.printrboard.close()
        self.synaptrons.close()

    def home(self):
        # First home 3 small motors for z axes
        self.homeZAxis(0)
        self.homeZAxis(1)
        self.homeZAxis(2)
        self.printrboard.sendSyncCmd("G28 Y\n")
        self.printrboard.sendSyncCmd("G28 X\n")
        return
        
    def homeZAxis(self, zNum):
        # To home the Z axes, move it all the way up,
        # wait for the limit switch to be engaged,
        # disable motor, reset position and setpoint,
        # re-enable motor
        print "Homing Z axis", zNum
        # TEMPORARY HACK
        self.synaptrons = santaFeSerial(self.synaptronsPort)
        address = self.ZAddressBase+zNum
        cmd = "{0},39,-7000\r\n".format(address)
        self.synaptrons.sendSyncCmd(cmd)
        # Better than sleeping, we should poll the controller
        # until the switch is engaged, with a timeout.
        time.sleep(5)
        self.synaptrons.sendSyncCmd("{0},03,22\r\n".format(address))
        self.synaptrons.sendSyncCmd("{0},05,0\r\n".format(address))
        self.synaptrons.sendSyncCmd("{0},39,0\r\n".format(address))
        self.synaptrons.sendSyncCmd("{0},03,6\r\n".format(address))
        # TEMPORARY HACK
        self.synaptrons.close()
        return

    def moveTo(self, pt):
        self.currentPosition = pt
        cmd = "G01 X{0[0]:.4} Y{0[1]:.4}\n".format(pt)
        self.printrboard.sendSyncCmd(cmd)
        self.dwell(1)

        time.sleep(0.5)

        # TEMPORARY HACK
        self.synaptrons = santaFeSerial(self.synaptronsPort)
        self.synaptrons.sendSyncCmd("54,39,{0}\r\n".format(int(pt[2]*self.ZCountsPerMM)))
        # TEMPORARY HACK
        self.synaptrons.close()

        time.sleep(0.5)
        
        # TEMPORARY HACK
        self.synaptrons = santaFeSerial(self.synaptronsPort)
        self.synaptrons.sendSyncCmd("55,39,{0}\r\n".format(int(pt[3]*self.ZCountsPerMM)))
        # TEMPORARY HACK
        self.synaptrons.close()

        time.sleep(0.5)

        # TEMPORARY HACK
        self.synaptrons = santaFeSerial(self.synaptronsPort)
        self.synaptrons.sendSyncCmd("56,39,{0}\r\n".format(int(pt[4]*self.ZCountsPerMM)))
        # TEMPORARY HACK
        self.synaptrons.close()

        time.sleep(0.5)

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

    def vacuum(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P12 S255\n"
        else:
            cmd = "M42 P12 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def air(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P11 S255\n"
        else:
            cmd = "M42 P11 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def smallPartManip(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P9 S255\n"
        else:
            cmd = "M42 P9 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def flyManip(self, onOff = False):
        if (onOff == True):
            cmd = "M42 P10 S255\n"
        else:
            cmd = "M42 P10 S0\n"
        self.printrboard.sendCmd(cmd)
        return

    def spinSmallPartManip(self, angle):
        cmd = "G01 E{0:.4}\n".format(angle)
        self.printrboard.sendCmd(cmd)
        return

    def spinFlyManip(self, angle):
        cmd = "G01 Z{0:.4}\n".format(angle)
        self.printrboard.sendCmd(cmd)
        return


# Set up close-up camera
def cameraInit(cam=1):
    print "Opening camera interface (", cam, ")"
    close = cv2.VideoCapture(cam)
    print "Setting properties"
    close.set(cv2.cv.CV_CAP_PROP_EXPOSURE, -4)
    close.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, 10)
    close.set(cv2.cv.CV_CAP_PROP_GAIN, 5)
    close.set(cv2.cv.CV_CAP_PROP_SATURATION, 12)
    close.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 2592)
    close.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1944)
    time.sleep(1)
    s, throwaway = close.read()
    if (s == False) or (close.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH) != 2592):
        close.release()
        print "Close cam init failed.", s
        return None
    else:
        return close


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

