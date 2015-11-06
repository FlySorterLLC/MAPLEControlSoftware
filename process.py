#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC

## Hardcoded program to pick & deposit flies

import cv2
import numpy as np
import time
import ConfigParser

import robotutil

def findFlies(self, image):
        
    # Copy just the stage part of the image
    self.maskedImage[self.maskIndex] = image[self.maskIndex]
    # Convert masked image to grayscale and threshold
    imgray = cv2.cvtColor(self.maskedImage,cv2.COLOR_BGR2GRAY)
    thresh = cv2.bitwise_not(cv2.threshold(imgray,140,255,cv2.THRESH_BINARY)[1])
    # Find contours in thresholded image
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                        
    flyCount = 0
    centers = np.zeros( (flyCount, 2) ).astype(np.float32)

    # Scoring only makes sense if there are flies to count
    if len(contours) >= 0:
                            
    # Loop through all the contours, calculate and keep track of centers
        for i, cntr in enumerate(contours):
            mmnts = cv2.moments(cntr)
            if mmnts['m00'] > 0:
                xBar = mmnts['m10'] / mmnts['m00']
                yBar = mmnts['m01'] / mmnts['m00']

                if ( mmnts['m00'] > self.areaThreshold ):
                    cv2.drawContours(image, cntr, -1, (0, 0, 255), thickness = -1)

                else:
                    centers = np.append(centers, [[xBar, yBar]], axis=0)
                    cv2.drawContours(image, cntr, -1, (255, 0, 0), thickness = 2)

    return centers
