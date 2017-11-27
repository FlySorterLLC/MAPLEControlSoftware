#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly Dispenser class file
import sys
import os
import numpy as np
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import flysorterSerial

class FlyDispenser:

    MaxThickness = 7.5
    Z2WorkingThickness = 4.0
    dispenserPort = None

    def __init__(self, dispenserPoint):

        self.dispenserPoint = dispenserPoint
        ## TODO: add serial class, instance of serial object
        ## and initialize dispenser (send "I")
        portList = flysorterSerial.availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            print "Trying port:", portDesc
            tempPort = flysorterSerial.serialDevice(portDesc)
            tempPort.sendCmd('V')
            time.sleep(1)
            r = tempPort.getSerOutput()
            print "Got reply: ", r
            if r.startswith("  V"):
                print "Port:", portDesc, "is dispenser."
                self.dispenserPort = tempPort
                self.dispenserPort.sendSyncCmd('I')
                break
            tempPort.sendCmd('\r\n')
            tempPort.close()

        if self.dispenserPort is None:
            print "Serial initialization failed."
        return

    def __del__(self):
        if self.dispenserPort is not None:
            self.dispenserPort.close()
        return

    # Returns 0 on success, 1 on timeout, 2 on failure
    def dispenseFly(self):
        # Send command ("F")
        self.dispenserPort.sendSyncCmd('F')
        # Get reply (to check whether fly was successfully dispensed or not)
        r = ""
        while r == "":
            r = self.dispenserPort.getSerOutput()
            time.sleep(0.25)
        s = r.rstrip("\r\n")
        #print "Reply from fly dispenser:", s
        if ( s == "f"):
            return 1
        elif ( s == "t"):
            return 0
        else:
            return 2

    def purge(self):
        # Send command ("P")
        self.dispenserPort.sendSyncCmd('P')
        r = ""
        while r == "":
            r = self.dispenserPort.getSerOutput()
            time.sleep(0.25)
        s = r.rstrip("\r\n")
        print "Reply from fly dispenser:", s
        if ( s == "f"):
            return 0
        else:
            return 1
