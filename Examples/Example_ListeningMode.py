##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: Example_ListeningMode.py
#  Description: Example routine calling highest-level functions.
#  Send an instruction via email using the apropriate format (see remoteOperation.py for example) to start, e.g., virgining routine remotely.
#  Cycling listenMode() permits MAPLE to perform repeated remote instructions.


## Dependencies
import cv2
import numpy as np
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
import remoteOperation as remote

# Import workspace that has the modules we're using.
import ExampleSocialWorkspace

# Now we can reference the modules in ExampleSocialWorkspace, like:
#   The dispenser object is ExampleSocialWorkspace.SocialWorkspace['dispenser1']
#   The social arena object is ExampleSocialWorkspace.SocialWorkspace['socialArena']

# Set amount of listenMode() cycles.
listeningTime = 60		# Cycles of engaging listeningMode(). Duration is ~ 1m if no instruction was found.

#### Initialize MAPLE ####
robot = robotutil.MAPLE("MAPLE.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()

## Starts main listening mode routine
timeelapsed = 0
while ( timeelapsed <= listeningTime ):
    mail = remote.listenMode(statusURL="", duration=60, listenInterval=2, robotEMailAccount='ExampleAccount@gmail.com', PWrobotEMailAccount='ExamplePassword')       # Engages listening mode for 1 minute.
    if mail != None:        # Performs instruction if keywords found.
        robot.home()
        remote.doInstruct(robot=robot, instruction=mail['instruct'], mailfrom=mail['from'], values=mail['values'], arena=ExampleSocialWorkspace.SocialWorkspace['socialArena'], FlyPlate=ExampleSocialWorkspace.SocialWorkspace['plate1'], turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz, vacZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz, arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii, dispenser=ExampleSocialWorkspace.SocialWorkspace['dispenser1'], HomeZwd=44, HomeZdp=44)
        robot.home()
    timeelapsed = timeelapsed + 1
    print 'Cycles since start:', timeelapsed
