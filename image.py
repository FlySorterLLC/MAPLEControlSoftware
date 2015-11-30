#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Hardcoded program to pick & deposit flies

import cv2
import numpy as np
import time
import ConfigParser

import robotutil

# Image Capture 
webcam = cv2.VideoCapture(2)

webcam.read()

close = cv2.VideoCapture(0)
close.set(cv2.cv.CV_CAP_PROP_EXPOSURE, -4)
close.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, 10)
close.set(cv2.cv.CV_CAP_PROP_GAIN, 5)
close.set(cv2.cv.CV_CAP_PROP_SATURATION, 12)
close.set(cv2.cv.CV_CAP_PROP_HUE, 13)
close.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 2592)
close.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 1944)

cv2.namedWindow("Video Capture")

key = -1

while key != 27:
    s, img = close.read()
    cv2.imshow("foo", img)    
    key = cv2.waitKey(10)

cv2.destroyAllWindows()
close.release()
