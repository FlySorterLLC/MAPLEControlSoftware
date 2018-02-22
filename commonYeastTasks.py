##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

import random as rand
import time
import numpy as np
import cv2

# Highest order commands for yeast colony manipulation

def applicatorEquip(robot, YeastApplicatorPlate, ID, applicatorZ=22.5, vacDur=500):
    coordX = YeastApplicatorPlate.getApplicatorCoords(ID)[0]
    coordY = YeastApplicatorPlate.getApplicatorCoords(ID)[1]
    robot.moveToSpd(pt=[float(coordX), float(coordY), 0, 0, 0], spd=5000)
    robot.dwell(t=1)
    robot.smallPartManipVac(True)
    robot.dwell(t=1)
    trylower = robot.lowerCare(z=applicatorZ, descendZ=5, retreatZ=5)      # lower onto applicator ID
    if trylower['limit'] == 1:
        robot.dwell(10)
        robot.homeZ2()
        for tries in range(0,1):
            robot.moveToSpd(pt=[float(coordX)-(tries+1/2), float(coordY), 0, 0, applicatorZ-19], spd=2000)
            robot.dwell(10)
            trylower = robot.lowerCare(z=applicatorZ, descendZ=10, retreatZ=10)
            if trylower['limit'] == 0:
                break
            else:
                robot.homeZ2()
    if trylower['limit'] == 1:
        for tries in range(0,1):
            robot.moveToSpd(pt=[float(coordX)+(tries+1/2), float(coordY), 0, 0, applicatorZ-19], spd=2000)
            robot.dwell(10)
            trylower = robot.lowerCare(z=applicatorZ, descendZ=10, retreatZ=10)
            if trylower['limit'] == 0:
                break
            else:
                robot.homeZ2()
    if trylower['limit'] == 1:
        for tries in range(0,1):
            robot.moveToSpd(pt=[float(coordX), float(coordY)-(tries+1/2), 0, 0, applicatorZ-19], spd=2000)
            robot.dwell(10)
            trylower = robot.lowerCare(z=applicatorZ, descendZ=10, retreatZ=10)
            if trylower['limit'] == 0:
                break
            else:
                robot.homeZ2()
    if trylower['limit'] == 1:
        for tries in range(0,1):
            robot.moveToSpd(pt=[float(coordX), float(coordY)+(tries+1/2), 0, 0, applicatorZ-19], spd=2000)
            robot.dwell(10)
            trylower = robot.lowerCare(z=applicatorZ, descendZ=10, retreatZ=10)
            if trylower['limit'] == 0:
                break
            else:
                robot.homeZ2()
    robot.dwell(t=vacDur)
    robot.homeZ2()        # retrieve applicator
    robot.moveToSpd(pt=[415, 247, 0, 0, 0, 0], spd=3000)       # move safely into neutral position

def applicatorDiscard(robot, discardX= 438, discardY = 116, discardZ=50, airBurst=2, airDur=1000):
    robot.moveToSpd(pt=[float(discardX), float(discardY), 0, 0, 0], spd=5000)
    robot.dwell(t=1)
    robot.lowerCare(z=discardZ, descendZ=10, retreatZ=10)      # lower into discard receptacle
    robot.smallPartManipVac(False)
    for b in range(0,airBurst):
        robot.flyManipAir(True)
        robot.dwell(t=airDur)
        robot.flyManipAir(False)
        robot.dwell(t=airDur/10)
    robot.moveRel(pt=[0, 0, 0, 0, -discardZ])        # 

def applicatorTest(robot, testX=415, testY=255, testZ=13):
    robot.moveToSpd(pt=[float(testX), float(testY), 0, 0, 0], spd=5000)
    robot.dwell(t=1)
    test = robot.lowerCare(z=testZ, descendZ=10, retreatZ=10)      # test whether applicator is equipped
    robot.homeZ2()
    if test['limit'] == 1:
        print 'Applicator is equipped.'
    else:
        print 'Applicator is not equipped.'
    return test['limit']

def lidWithdraw(robot, YeastArena, ID, adjZ=0):
    coordX = YeastArena.getSPCoords(ID)[0]
    coordY = YeastArena.getSPCoords(ID)[1]
    coordZ = YeastArena.SPz+adjZ
    robot.moveToSpd(pt=[float(coordX), float(coordY), 0, 0, 0], spd=5000)
    robot.dwell(1)
    robot.moveToSpd(pt=[float(coordX), float(coordY), coordZ, 0, 0], spd=3000)
    robot.dwell(1)
    robot.flyManipVac(True)
    robot.dwell(300)
    robot.moveRel(pt=[0, 0, -coordZ+10, 0, 0])      # so lid does not fall off

def lidPlace(robot, YeastArena, ID):
    coordX = YeastArena.getSPCoords(ID)[0]
    coordY = YeastArena.getSPCoords(ID)[1]
    coordZ = YeastArena.SPz
    robot.moveToSpd(pt=[float(coordX), float(coordY), 10, 0, 0], spd=5000)
    robot.dwell(1)
    robot.moveToSpd(pt=[float(coordX), float(coordY), coordZ-1, 0, 0], spd=4000)
    robot.dwell(1)
    robot.flyManipVac(False)
    robot.smallPartManipAir(True)
    robot.dwell(100)
    robot.moveRel(pt=[0, 0, -coordZ+1, 0, 0])
    robot.dwell(1)
    robot.smallPartManipAir(False)

def lidTest(robot, YeastArena, ID):
    coordX = YeastArena.getArenaCoords(ID)[0]
    coordY = YeastArena.getArenaCoords(ID)[1]
    coordZ = YeastArena.SPz-20
    robot.moveToSpd(pt=[float(coordX), float(coordY), 10, 0, 0], spd=5000)
    robot.dwell(1)
    test = robot.lowerCare(z=coordZ+1, descendZ=10, retreatZ=10)      # test whether lid is present
    robot.homeZ2()
    if test['limit'] == 1:
        print 'Lid detected.'
    else:
        print 'Lid not detected.'
    return test['limit']

def lidCheck(robot):
    vacSens = robot.smoothie.sendCmdGetReply("M109\n").split(' ')
    print vacSens
    return vacSens

def colonyProbe(robot, YeastArena, ID, colonyX, colonyY, probeT=100, skipAnchor=False, skipLidProbe=True, agarZ=0):
    arenaX = YeastArena.getArenaCoords(ID)[0]
    arenaY = YeastArena.getArenaCoords(ID)[1]
    if agarZ == 0:
        agarZ = YeastArena.agarZ
    spZ = YeastArena.SPz
    if skipAnchor == False:
        robot.moveToSpd(pt=[float(arenaX-10), float(arenaY-10), 10, 0, 0], spd=5000)
        robot.dwell(1)
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(colonyX), CurCoord[1], 10, 0, CurCoord[4]], spd=3000)
    time.sleep(0.1)
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(colonyX), float(colonyY), 10, 0, CurCoord[4]], spd=3000)
    time.sleep(0.1)
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[CurCoord[0], CurCoord[1], 10, 0, agarZ-10], spd=3000)
    if skipLidProbe==False:
        trylower = robot.lowerCare(z=spZ-22, descendZ=10, retreatZ=10)     # probe for closed lid
        if trylower['limit'] == 1:
            robot.homeZ2()
            print 'Check yeast arena', ID, 'lid.'
            return trylower
        robot.dwell(100)
    reset = robot.lowerCare(z=agarZ, descendZ=5, retreatZ=5)       # move applicator towards agar surface
    robot.dwell(probeT)
    if reset['limit'] ==1:
        robot.homeZ2()
    else:
        robot.moveRel(pt=[0, 0, 0, 0, -10])
    time.sleep(0.1)

def getLogoCoord(ID):
        # M
    Logo = [    [310, 156], [310,152],[310, 148], [310,146],[310, 142], [310,138],
        [309,140],  [307,144],[305,148], [303,144],[301, 140],
        [300,138], [300,142],[300,146],[300,148],[300,152],[300,156],
        # A
        [295,156], [295,152],[295, 148], [295,146],[295, 142], [295,138],
        [293,138],[290,138],[287,138],
        [285,138], [285, 142], [285, 146],[285, 148], [285,152],[285, 156],
        [293, 148], [290, 148], [287, 148],
        # P
        [282,156], [282,152],[282, 148], [282,146],[282, 142], [282,138],
        [280,138], [277,138],[274,138],
        [280,148],[277,148],[274,148],
        [274,148], [274,144],[274, 142], [274,138],
        # L
        [269,156], [269,152],[269, 148], [269,146],[269, 142], [269,138],
        [267,156],[264,156],[261,156],
        # E
        [256,156], [256,152],[256, 148], [256,146],[256, 142], [256,138],
        [253,156],[250,156],[247,156],
        [253,147],[250,147],[247,147],
        [253,138],[250,138],[247,138]    ]
    return Logo[ID]

def getLogoCoordFull(ID):
        # M
    Logo = [    [310, 156],[310, 154], [310,152],[310, 150],[310, 148], [310,146], [310,144],[310, 142],[310, 140], [310,138],
        [309,140], [308,142], [307,144],[306,146],[305,148], [304,146], [303,144],[302, 142],[301, 140],
        [300,138], [300,140], [300,142],[300,144],[300,146],[300,148],[300,150],[300,152],[300,154],[300,156],
        # A
        [295,156], [295, 154], [295,152],[295, 150],[295, 148], [295,146], [295,144],[295, 142],[295, 140], [295,138],
        [293,138], [291,138],[289,138],[287,138],
        [285,138], [285,140], [285, 142], [285,144],[285, 146],[285, 148], [285,150], [285,152],[285, 154],[285, 156],
        [293, 148], [291, 148], [289, 148], [287, 148],
        # P
        [282,156], [282, 154], [282,152],[282, 150],[282, 148], [282,146], [282,144],[282, 142],[282, 140], [282,138],
        [280,138], [278,138],[276,138],[274,138],
        [280,148], [278,148],[276,148],[274,148],
        [274,148], [274,146], [274,144],[274, 142],[274, 140], [274,138],
        # L
        [269,156], [269, 154], [269,152],[269, 150],[269, 148], [269,146], [269,144],[269, 142],[269, 140], [269,138],
        [267,156], [265,156],[263,156],[261,156],
        # E
        [256,156], [256, 154], [256,152],[256, 150],[256, 148], [256,146], [256,144],[256, 142],[256, 140], [256,138],
        [254,156], [252,156],[250,156],[248,156],
        [254,147], [252,147],[250,147],[248,147],
        [254,138], [252,138],[250,138],[248,138]    ]
    return Logo[ID]

def imgArena(robot, YeastArena, ID, light=0):
    coordX = YeastArena.getCamCoords(ID)[0]
    coordY = YeastArena.getCamCoords(ID)[1]
    coordZ = YeastArena.camsharpz
    robot.moveToSpd(pt=[float(coordX), float(coordY), 10, coordZ, 0], spd=5000)
    if light != 0:
        time.sleep(0.1)
        robot.light(True)
        time.sleep(0.1)
    img = robot.captureImage()
    if light != 0:
        robot.light(False)
    return img

def detectColony(robot, YeastArena, ID):
    img_width=1280
    img_height=960
    coordX = YeastArena.getCamCoords(ID)[0]
    coordY = YeastArena.getCamCoords(ID)[1]
    coordZ = YeastArena.camsharpz
    robot.moveToSpd(pt=[float(coordX), float(coordY), 10, coordZ, 0], spd=5000)
    robot.dwell(1)
    robot.light(True)
    time.sleep(0.2)
    img = robot.captureImage()
    robot.light(False)
    # img = cv2.resize(img, (1280, 960))
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,2.6,50, param1=139,param2=170,minRadius=8,maxRadius=30)
    #for i in range(0,len(circles)):
        #cv2.circle(thresh, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
    #cv2.imshow("output", thresh)
    #cv2.waitKey(0)
    circles = robot.findOpening(img, slowmode=False, MAX_SIZE=30, MIN_SIZE=8, startp1=139, startp2=170, startp3=2.6, imgshow=0)
    x = circles[0][0]
    y = circles[0][1]
    imgmid = [img_width/2, img_height/2]
    xrel = imgmid[0] - x
    yrel = imgmid[1] - y
    return xrel, yrel

def streakColony(robot, YeastArena, ID, agarZ=0):
    arenaX = YeastArena.getArenaCoords(ID)[0]
    arenaY = YeastArena.getArenaCoords(ID)[1]
    if agarZ == 0:
        agarZ = YeastArena.agarZ
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(arenaX), CurCoord[1], 10, 0, CurCoord[4]], spd=3000)
    time.sleep(0.2)
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(arenaX), float(arenaY), 10, 0, CurCoord[4]], spd=3000)
    time.sleep(0.2)
    robot.moveRel([25, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, 21, 0, 0, 0])
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[CurCoord[0], CurCoord[1], 10, 0, agarZ-10], spd=1000)
    robot.lowerCare(z=agarZ, descendZ=5, retreatZ=5)
    time.sleep(0.1)
    robot.moveRel([-50, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, 3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([47, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, 3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-44, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, 3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([41, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, 3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-38, 0, 0, 0, 0])
    time.sleep(0.1)

    robot.moveRel([0, 0, 0, 0, -10])
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(arenaX), CurCoord[1], 10, 0, CurCoord[4]], spd=3000)
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[float(arenaX), float(arenaY), 10, 0, CurCoord[4]], spd=3000)
    time.sleep(0.2)
    robot.moveRel([31, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -7, 0, 0, 0])
    CurCoord = robot.getCurrentPosition()
    robot.moveToSpd(pt=[CurCoord[0], CurCoord[1], 10, 0, agarZ-10], spd=1000)
    robot.lowerCare(z=agarZ, descendZ=5, retreatZ=5)
    time.sleep(0.1)
    robot.moveRel([-62, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([59, 0, 0, 0, 0])
    time.sleep(0.1)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-56, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([53, 0, 0, 0, 0])
    time.sleep(0.1)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-50, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([47, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-44, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([41, 0, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([-38, 0, 0, 0, 0])
    time.sleep(0.1)
    robot.moveRel([0, -3, 0, 0, 0])
    time.sleep(0.2)
    robot.moveRel([35, 0, 0, 0, 0])