#! /usr/bin/env python

## Copyright (c) 2015, FlySorter, LLC
## Machine Vision code to take images of flies and return the locations (in pixels) of
## all flies in the image. 
## Also draws the contours, bounding rectangles, and centroids of each fly

## By: Will Long
## MRU: Nov 19 2015

import cv2
import numpy as np
import time
import math
import ConfigParser
from matplotlib import pyplot as plt
from itertools import combinations

class imageProcess:
    
    #findFlies takes an image and executes some preliminary editing and 
    #thresholding in order to better identify flies.
    #Specifically, it thresholds images for objects within a color range
    
    #Takes an image file as an argument and returns nothing
    def findFlies(self, image):
        
        frame = cv2.imread(image)
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0,30,30])
        upper_red = np.array([255,250,100])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        # Writes the result onto a new file called 'res.bmp'
        cv2.imwrite('res.bmp', mask)

    #returnFlies takes a processed image 'res.bmp' and:
    # 1. Draws green contours around all flies
    # 2. Draws a red bounding rectangle around each fly
    # 3. Returns the center of the bounding rectangle in pixel coordinates
    #    (x,y) with the origin at the top left hand side and draws a blue dot
    #takes image file as an argument and returns an array of all coordinate pairs
    
    def returnFlies(self,image):
        
        img = cv2.imread(image)
        img = cv2.resize(img,(400,500))
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,gray = cv2.threshold(gray,127,255,0)
        gray2 = gray.copy()
        mask = np.zeros(gray.shape,np.uint8)

        contours, hier = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        keypoints = []
        
        for cnt in contours:

            if 500<cv2.contourArea(cnt)<3000:
                cv2.drawContours(img,[cnt],0,(0,255,0),2)
                cv2.drawContours(mask,[cnt],0,255,-1)
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                keypoints.append((x+w/2,y+h/2))
            
        for a in range(len(keypoints)):
            cv2.circle(img, keypoints[a], 3, 255,-1)

        cv2.imwrite("res2.bmp", img)
        return keypoints

    def execute(self, image):
        self.findFlies(image)
        print(self.returnFlies("res.bmp"))
        
#------------Test Programs ----------
image = input("Image to Process:  ")
a = imageProcess()
a.execute(image)

cv2.imshow('Processed Image', cv2.imread("res2.bmp"))
cv2.waitKey(0)
cv2.destroyAllWindows()