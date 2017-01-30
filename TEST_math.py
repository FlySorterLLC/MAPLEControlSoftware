
import numpy as np
import time
import math
import matplotlib.pyplot as plt
import robotutil
import serial

robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.home()
robot.moveTo(pt=[348, 250, 0, 0, 0])
a = 0
i = 1
for j in range(10):
	for i in range(54):
		robot.moveZ(pt=[348, 250, 0, 0, 1+i])
		b = robot.getLimit()
		if b == 1:
			print 'break'
			break
	robot.moveTo(pt=[348, 250, 0, 0, 0])