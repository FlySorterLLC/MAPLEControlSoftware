import cv2
import numpy as np
import time
import robotutil
import math
import matplotlib.pyplot as plt

robot = robotutil.santaFe("SantaFe.cfg")

robot.home()
robot.moveXY(pt=[305, 240])
robot.dwell(t=500)
XYpos = robot.getCurrentPosition()
robot.moveZ(pt=[XYpos[0],XYpos[1],0,45,0])

iterations = 500
tempdeg = np.arange(iterations)
for i in range(iterations):
	tempdeg[i] = robot.findDegs(slowmode=True)

x = np.arange(iterations)
plt.plot(x, tempdeg)
plt.show()