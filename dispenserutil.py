#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Fly Dispenser class file

import numpy as np
import robotutil
import time
import os
import math
import cv2
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

class FlyDispenser:

    MaxThickness = 7.5
    Z2WorkingThickness = 4.0
    dispenserPort = None

    def __init__(self, dispenserPoint=9600):

        #self.dispenserPoint = dispenserPoint
        ## TODO: add serial class, instance of serial object
        ## and initialize dispenser (send "I")
        portList = robotutil.availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            print "Trying port:", portDesc
            tempPort = robotutil.santaFeSerial(portDesc, dispenserPoint)
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
        reply = r.rstrip("\r\n")
        print "Reply from fly dispense:", reply     # 1:dispensed   0:no fly detected   2:fly stuck
        if ( reply == "f"):
            return 1
        elif ( reply == "t" ):
            return 0
        else:
            return 2