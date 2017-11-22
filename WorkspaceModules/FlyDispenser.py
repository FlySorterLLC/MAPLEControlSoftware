#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly Dispenser class file

import numpy as np
import robotutil
import time

class FlyDispenser:

    MaxThickness = 7.5
    Z2WorkingThickness = 4.0
    dispenserPort = None

    def __init__(self, dispenserPoint):

        self.dispenserPoint = dispenserPoint
        ## TODO: add serial class, instance of serial object
        ## and initialize dispenser (send "I")
        portList = robotutil.availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            print "Trying port:", portDesc
            tempPort = robotutil.serialDevice(portDesc)
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

    # Returns 0 on success, 1 on failure
    def dispenseFly(self):
        # Send command ("F")
        self.dispenserPort.sendSyncCmd('F')
        # Get reply (to check whether fly was successfully dispensed or not)
        r = ""
        while r == "":
            r = self.dispenserPort.getSerOutput()
            time.sleep(0.25)
        s = r.rstrip("\r\n")
        print "Reply from fly dispense:", s
        if ( s == "f"):
            return 0
        else:
            return 1
