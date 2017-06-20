import cv2
import numpy as np
import time
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
import smtplib
import email
import serial
import urllib2




robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.home()
robot.dispenseWithdraw(dispX=639.5, dispY=113, dispZ=34, dispiter=2)
#robot.arenaDeposit(camcoordX=349.5-44, camcoordY=269.3-6.3, camcoordZ=40, arenacoordX=349.5, arenacoordY=269.3, arenaRad=10.5, turnZ=49, airPos=50, airZ=50, closePos=180, airBurst=1, imgshow=0)
#robot.home()