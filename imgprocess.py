#! /usr/bin/env python

## Copyright (c) 2015, William Long
## Machine Vision code to take images of flies and return the locations (in pixels) of
## all flies in the image. 
## Also draws the contours, bounding rectangles, and centroids of each fly

## By: Will Long
## MRU: Oct 5 2016

import cv2
import numpy as np
import time
import math
import ConfigParser

#from matplotlib import pyplot as plt
#from itertools import combinations

reference = (40,15)   #pixel coordinate that represents a robot movement of (x = 40, y = 15)
PPMM = 28               #pixels per millimeter at the height of optimum resolution

padLocation = (1000, 3000, 0, 0, 0)     #location of top left hand corner of pad
padSize = (500, 500)                    #size of the pad in x, y
mazeLocation = (3000, 3000, 0, 0, 0)
imageSize = (1900/PPMM, 1900/PPMM) #(1900, 1900) in pixels

class imageProcess:
    
    #processImage takes an image and executes some preliminary editing and 
    #thresholding in order to better identify flies.
    #Specifically, it thresholds images for objects within a color range
    
    #Takes an image array as an argument and returns the image
    def processImage(self, image):
        
        frame = cv2.imread(image)
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0,0,0])
        
        upper_red = np.array([255,250,100])
        # upper_red = np.array([255,50,250])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        # Writes the result onto a new file called 'res.bmp'
        # cv2.imwrite('res.bmp', mask)      Probably don't need this any longer

        return res

    def findOpening(self, image):
        
        result = []
        MAX_SIZE = 60  # range of size in pixels of the circle lid hole
        MIN_SIZE = 35
        
        image = cv2.imread(image)
        output = image.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,1.3,300,
                                   param1=50,param2=70,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)

        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
        
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:

                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                # Radius constraints are for MATTED circle lids
                if not (x in range(270, 570) and y in range(150, 450)):
                	if x in range(140, 700) and y in range(20, 580):
	                    result.append((x,y))
	                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
	                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    
        cv2.circle(output,(420,300), 280, (0,255,0), 3, -1)
        cv2.circle(output,(420,300), 150, (0,255,0), 3, -1)
        cv2.imshow("Output", output)
        cv2.waitKey(0)
        
        return result

    def watershed(self, image):
    	img = cv2.imread(image)
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    	ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    	return thresh

    # returnFlies takes a processed image 'res.bmp' and:
    # 1. Draws green contours around all flies
    # 2. Draws a red bounding rectangle around each fly
    # 3. Returns the center of the bounding rectangle in pixel coordinates
    #    (x,y) with the origin at the top left hand side and draws a blue dot
    # takes image file as an argument and returns an array of all coordinate pairs
    
    def returnFlies(self,image, reference):
        
        img = image
        # img = cv2.resize(img,(400,500))     #have to resize image otherwise it's too big!
        
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,gray = cv2.threshold(gray,127,255,0)
        mask = np.zeros(gray.shape,np.uint8)

        #Use a simple OpenCV algorithm to locate contours in the image
        contours, hier = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        flyPixelXY = []      #list of fly coordinates in pixels
        
        for cnt in contours:

            if 1500<cv2.contourArea(cnt)<25000:
                cv2.drawContours(img,[cnt],0,(0,255,0),2)
                cv2.drawContours(mask,[cnt],0,255,-1)
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                flyPixelXY.append((x+w/2,y+h/2))
            
        for i in range(len(flyPixelXY)):
            cv2.circle(img, flyPixelXY[i], 7, 255,-1)
        
        cv2.circle(img, reference, 7, (0, 0, 255),-1) #draw a red dot at the reference point
        
        flyRobotXY = []    #fly coordinates in millimeters from the reference point
        
        for (x, y) in flyPixelXY:
            flyRobotXY.append(((x - reference[0])/PPMM, (y - reference[1])/PPMM))
            
        for (x, y) in flyRobotXY:
            cv2.circle(img, (reference[0] + x*PPMM, reference[1] + y*PPMM), 7, (0, 0, 255),-1)
        
        cv2.imwrite("res2.bmp", img)
        return flyRobotXY

    def config(self, image):
        img = cv2.imread(image)
        img = cv2.resize(img, (800,500))
        cv2.circle(img, (357, 410), 10, 255,-1)
        cv2.circle(img, (537, 410), 10, 255,-1)
        cv2.imshow('Configuration', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        
    def execute(self, image, reference):
        result = self.processImage(image)
        return self.returnFlies(result, reference)


# -------   Test Programs ------------
# a = imageProcess()


# cv2.imshow("Processed Lid", a.watershed("img11.png"))
print a.findOpening("temp_img.png")
# print a.findOpening("img1.png")
# print a.findOpening("img2.png")
# print a.findOpening("img3.png")
# print a.findOpening("img4.png")
# print a.findOpening("img5.png")
# print a.findOpening("img6.png")
# print a.findOpening("img7.png")
# print a.findOpening("img8.png")
# print a.findOpening("img9.png")
# print a.findOpening("img10.png")