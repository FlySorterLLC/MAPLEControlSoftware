#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly Dispenser class file

import os
import math
import cv2
import numpy as np
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
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
        robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
        robot.dwell(50)
        limit = robot.lowerCare(dispZ, descendZ=6, retreatZ=12)
        if onlyifsure == 0:
            print 'Depositing even if fly may be stuck.'
        if limit != 1:
            robot.smallPartManipVac(True)
            for iter in range(dispiter):
                dispsuccess = robot.dispenseFly()
                if dispsuccess == 1:
                    print 'Fly dispensed.'
                    robot.dwell(2000)
                    robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
                    return dispsuccess
                elif dispsuccess == 0:
                    robot.smallPartManipVac(False)
                elif dispsuccess == 2 and onlyifsure == 1:
                    robot.smallPartManipVac(False)
                elif dispsuccess == 2 and onlyifsure == 0:
                    print 'Quantum fly in tunnel - better safe than sorry protocol commencing.'
                    robot.dwell(3000)
                    robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
        elif limit == 1:
            print 'Possible misallignment. Check fly dispenser.'
            robot.home()
            return None
        robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
        return dispsuccess
