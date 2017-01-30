import cv2
import numpy as np
import time
import math
import matplotlib.pyplot as plt

midpoint = [348, 250]
r = 11
deg = np.ones((360,2))
print deg
print len(deg)
endpos = 2
for x in range(0,len(deg)):
	deg[x,:] = (math.sin(endpos*math.pi/360*x)*r)+midpoint[0], (math.cos(endpos*math.pi/360*x)*r)+midpoint[1]
print deg
plt.scatter(deg[:,0], deg[:,1])
plt.show()