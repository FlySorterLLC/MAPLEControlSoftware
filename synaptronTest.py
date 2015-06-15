#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

import cv2
import numpy as np
import time
import ConfigParser

import robotutil

portName = "COM5"
addressRange = (54, 55, 56)
ZCountsPerMM = 6.0*29.86*10.4/25.4

serialPort = robotutil.santaFeSerial(portName, 9600, 0.01)

print "Homing"

for address in addressRange:
    print "Address", address
    serialPort.sendSyncCmd("{0},39,-7000\r\n".format(address))
    time.sleep(3)
    serialPort.sendCmd("{0},03,22\r\n".format(address))
    serialPort.sendCmd("{0},05,0\r\n".format(address))
    serialPort.sendCmd("{0},39,0\r\n".format(address))
    serialPort.sendCmd("{0},03,6\r\n".format(address))

time.sleep(0.25)

print "Moving"
for x in range(1, 10):
    for address in addressRange:
        print address, x
        serialPort.sendSyncCmd("{0},39,{1}\r\n".format(address, int(x*ZCountsPerMM)))
        time.sleep(0.5)

serialPort.close()
print "Done!"
