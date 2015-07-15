class santaFe_v2: # robot class for doing stuff with the machine
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
