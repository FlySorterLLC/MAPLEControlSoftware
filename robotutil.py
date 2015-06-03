import cv2
import numpy as np
import sys
import glob
import serial

class santaFeSerial:
    """Serial class for robot device."""
    
    WaitTimeout = 10

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
        return output

    def waitForOK(self):
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
        if output != 'ok\r\n':
            print "Got something other than 'ok' from serial: ", ':'.join(x.encode('hex') for x in output)

    # Send a command to the device via serial port
    # Asynchronous by default - doesn't wait for reply
    def sendCmd(self, cmd):
        self.ser.write(cmd)
        self.ser.flush()
        self.getSerOutput()

    # Send a command to the device via serial port
    def sendSyncCmd(self, cmd):
        serOutput = self.getSerOutput()
        while serOutput != '':
            serOutput = self.getSerOutput()
        self.ser.write(cmd)
        self.ser.flush()
        self.waitForOK()

    # Send a command and return the reply
    def sendCmdGetReply(self, cmd):
        serOutput = self.getSerOutput()
        while serOutput != '':
            serOutput = self.getSerOutput()
        self.ser.write(cmd)
        self.ser.flush()
        return self.getSerOutput()

    def sendFile(self, filename):
        return 1


class santaFe:
    """Class for fly manipulation robot."""

    # Constants
    safeZHeight = 40

    pxPerMM = np.float32([ 2592.0/7.0, 1944.0/5.4 ])
    imageCenter = np.float32( [2592.0/2.0, 1944.0/2.0 ])

    isInitialized = False
    
    def __init__(self, camIndex):
        self.printrboard = None
        self.synaptrons = None
        # Search serial ports, look for motor control board and printrboard
        portList = availablePorts()
        for portDesc in portList:
            print "Trying port", portDesc
            tempPort = santaFeSerial(portDesc)
            if tempPort.sendCmdGetReply("M115\r\n").startswith("FIRMWARE_NAME"):
                print "Port", portDesc, "is printrboard."
                self.printrboard = tempPort
                break
            if tempPort.sendCmdGetReply("54,00,\r\n").startswith("54"):
                print "Port", portDesc, "is motor controller."
                self.synaptrons = tempPort
                break
            tempPort.close()

        if self.printrboard is None:
            print "Printrboard not found"
        if self.synaptrons is None:
            print "Synaptrons not found"
        print self.synaptrons.sendCmdGetReply("54,00,\r\n")
        #self.cam = cameraInit(camIndex)
        #if self.cam == None:
        #    print "Camera init fail."
        #    self.serialPort.close()
        #    return

        self.isInitialized = True

    def release(self):
        #self.cam.release()
        self.printrboard.close()
        self.synaptrons.close()

    def home(self):
        cmd = "G28 Y\n"
        self.printrboard.sendSyncCmd(cmd)
        cmd = "G28 X\n"
        self.printrboard.sendSyncCmd(cmd)
        # Now home 
    
    def moveTo(self, pt):
        cmd = "G01 Z{0}\n".format(self.safeZHeight)
        self.printrboard.sendCmd(cmd)
        cmd = "G01 X{0[0]:.4} Y{0[1]:.4} Z{0[2]:.4}\n".format(pt)
        self.printrboard.sendCmd(cmd)

    def unsafeMove(self, pt):
        cmd = "G01 X{0[0]:.4} Y{0[1]:.4} Z{0[2]:.4}\n".format(pt)
        self.printrboard.sendCmd(cmd)
    
    def dwell(self, t):
        cmd = "G04 P{0}\n".format(t)
        self.printrboard.sendSyncCmd(cmd)

    def light(self, LEDs = 'vis', onOff = False):
        if (LEDs == 'vis'):
            if ( onOff == True ):
                cmd = "M106\n"
            else:
                cmd = "M107\n"
            self.printrboard.sendCmd(cmd)
        else:
            print "Don't understand how to turn on LED:", LEDs
            return

    def vacuum(self, onOff = False, port = 0):
        if (port == 0):
            if (onOff == True):
                cmd = "M42 P14 S255\n"
            else:
                cmd = "M42 P14 S0\n"
        elif (port == 1):
            if (onOff == True):
                cmd = "M42 P15 S255\n"
            else:
                cmd = "M42 P15 S0\n"
        self.printrboard.sendCmd(cmd)



# Set up close-up camera
def cameraInit(cam=1):
    print "Opening close-up cam", cam
    close = cv2.VideoCapture(cam)
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
