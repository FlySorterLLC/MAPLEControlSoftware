## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: Example_ListeningMode.py
#  Description: Example routine calling highest-level functions in robotutil.py.
#  Send an instruction via email using the apropriate format (see robotutil.py for example) to start, e.g., virgining routine remotely.
#  Workspace dimenions are assumed to be maximal (as defined in robotutil.py);
#  reads Arena_Coordinates.cfg for behavioral and housing module coordinates and dimensions (Alternatively uses in-script defined variables).
#  Arena sequential order is determined by line-wise order of coordinates in Arena_Coordinates.cfg OR from list-wise highest to lowest X and Y coordinates as determined in-script (X and Y coordinates at the same order are considered a match in the latter case).



## Dependencies
import cv2
import numpy as np
import time
import robotutil
import math
import random as rand
import matplotlib.pyplot as plt
import ConfigParser
from datetime import datetime
import remoteOperation
import commonFlyTasks

#### BEGIN PGM ####
coordfromcfg = False       # Set True to read Arena_Coordinates.cfg file for coordinates
robot = robotutil.santaFe("SantaFe.cfg")
robot.smoothie.sendCmd("M999")
robot.flyManipAir(False)
robot.smallPartManipVac(False)
robot.home()


### Import workspace that has the modules we're using.
import ExampleSocialWorkspace

# Now we can reference the modules in ExampleSocialWorkspace, like:
#   The dispenser object is ExampleSocialWorkspace.SocialWorkspace['dispenser1']
#   The social arena object is ExampleSocialWorkspace.SocialWorkspace['socialArena']
#

## Starts main listening mode routine
timeelapsed = 0
while ( timeelapsed <= listeningTime ):
    mail = remoteOperation.listenMode()       # engages listening mode for 1 minute
    if mail != None:        # performs instruction
        robot.home()
        remoteOperation.doInstruct(robot, ExampleSocialWorkspace.SocialWorkspace['dispenser1'], instruction=mail['instruct'], mailfrom=mail['from'], values=mail['values'], CamX=CamX, CamY=CamY, CamZ=camsharpz, ManipX=ManipX, ManipY=ManipY, turnZ=POIz, vacZ=Vacz, arenaRad=10.5, HomeX=FlyHomeX, HomeY=FlyHomeY, HomeZwd=homezwd, HomeZdp=homezdepos)
        robot.home()
    timeelapsed = timeelapsed + 1       # in minutes
    print 'time since start:', timeelapsed
