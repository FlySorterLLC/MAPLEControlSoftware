## Copyright (c) 2015, FlySorter LLC
##
## This software is licensed under the GPL v2.0
#
#  File: remoteutil.py
#  Description: Contains classes and functions used to remotely control MAPLE


## Dependencies
import os
import math
import cv2
import numpy as np
import sys
import traceback
import glob
import serial
import time
import ConfigParser
import random as rand
import robotutil
sys.path.append(os.path.join(os.path.dirname(__file__), 'WorkspaceModules'))
import FlyPlate
import SocialArena
import FlyDispenser

class combinedfun:
    def __init__(self, startWellPoint, endWellPoint, anchorX, anchorY):
        robot = robotutil.santaFe("SantaFe.cfg")
        home = FlyPlate(startWellPoint, endWellPoint)
        arena = SocialArena(anchorX, anchorY)
        dispenser = FlyDispenser()

    # Tries to dispense and deposit all newly hatched flies into home plate / saves how many flies were successfully dispensed for longer processing
    def dispenseHIfHatched(self, homecoordX, homecoordY, dispX, dispY, dispZ, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, maxconsecstuck=4):
        nDispensed = carryovernDispensed
        IsHatched = 1
        consecStuck = 0
        while IsHatched == 1 or (IsHatched == 2 and onlyifsure == 0) and not consecStuck > maxconsecstuck:
            IsHatched = dispenser.dispenseWithdraw(dispX=dispX, dispY=dispY, dispZ=dispZ, dispiter=dispiter, onlyifsure=onlyifsure)
            if IsHatched == 1:
                home.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
                nDispensed = nDispensed+1
            elif IsHatched == 2 and onlyifsure == 0:
                home.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
                nDispensed = nDispensed +1
                consecStuck = consecStuck +1
            else:
                break
                print 'No fly dispensed.'
        print 'Dispensed', nDispensed, 'flies total.'
        return nDispensed

    # Repeatedly collect successfully dispensed flies (newly hatched) and deposit into consecutive single wells in the housing module
    # Not accurate loop time-wise but ensures a minimum amount of time tried collecting. All times in ms.
    def collectHatchedForT(self, homecoordX, homecoordY, dispX, dispY, dispZ, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, collectT=3600, collectInt=60, maxconsecstuck=4):
        for n in range(collectT/collectInt):
            carryovernDispensed = self.dispenseHIfHatched(homecoordX=homecoordX, homecoordY=homecoordY, dispX=dispX, dispY=dispY, dispZ=dispZ, onlyifsure=onlyifsure, carefulZ=9, vacBurst=vacBurst, homeZ=homeZ, dispiter=dispiter, carryovernDispensed=carryovernDispensed, maxconsecstuck=maxconsecstuck)
            time.sleep(collectInt)

    # Visual representation according to workspace and module dimensions (Hardcoded due to maximum workspace length used in current experiment)
    def visualizeInstruct(self, HXcoords, HYcoords, Xcoords, Ycoords, POIrad, dispx, dispy, instN, instHN=0, instdispN=0, start='home', ArenaSide='L'):
        height = max(HYcoords)*2
        width = max(HXcoords)*2
        background = np.zeros((height,width,3), np.uint8)
        background[:,:] = (170, 170, 170)
        # Robot home
        cv2.rectangle(background, (0, 0), (50, 50), (125, 125, 125), -1)
        # Tray outline
        cv2.rectangle(background, (min(Xcoords)-POIrad*1.5, min(Ycoords)-POIrad*1.5), (max(Xcoords)+POIrad*1.5, max(Ycoords)+POIrad*1.5), (0, 0, 0), -1)
        # Arenas
        for ncirc in range(len(Xcoords)):
            cv2.circle(background, (Xcoords[ncirc], Ycoords[ncirc]), int(POIrad), (255, 255, 255), 1)
            cv2.line(background, (Xcoords[ncirc], Ycoords[ncirc]-POIrad*0.75), (Xcoords[ncirc], Ycoords[ncirc]+POIrad*0.75), (255,255,255), thickness=1, lineType=1, shift=0)
        # Fly home outline
        cv2.rectangle(background, (min(HXcoords)-POIrad/2*1.5, min(HYcoords)-POIrad/2*1.5), (max(HXcoords)+POIrad/2*1.5, max(HYcoords)+POIrad/2*1.5), (55, 55, 55), -1)
        # Fly homes
        for ncirc in range(len(HXcoords)):
            cv2.circle(background, (HXcoords[ncirc], HYcoords[ncirc]), int(POIrad)/3, (255, 255, 255), 1)
        # Dispenser
        cv2.circle(background, (int(dispx), int(dispy)), int(POIrad), (155, 155, 155), -1)
        cv2.circle(background, (int(dispx), int(dispy)), int(POIrad)/2, (0, 0, 0), 2)

        # Visualize instruction (flip text before entire image is flipped for easier mapping)
        mask =  np.zeros((height,width,3), np.uint8)
        if start == 'arena':        # Considers fly in arena the 'actual' ID
            for ncirc in range(len(instN)):
                if instN[ncirc] <= 9:
                    if ArenaSide =='L':
                        cv2.putText(mask, str(instN[ncirc]), ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
                    elif ArenaSide =='R':
                        cv2.putText(mask, str(instN[ncirc]), ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
                else:
                    if ArenaSide =='L':
                        cv2.putText(mask, str(instN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                        cv2.putText(mask, str(instN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                    elif ArenaSide =='R':
                        cv2.putText(mask, str(instN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                        cv2.putText(mask, str(instN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]]+POIrad*0.06), int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
        elif start == 'home':       # Considers fly in home to be the start ID
            for ncirc in range(len(instHN)):
                if instHN[ncirc] <= 9:
                    if ArenaSide =='L':
                        cv2.putText(mask, str(instHN[ncirc]), ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
                    elif ArenaSide =='R':
                        cv2.putText(mask, str(instHN[ncirc]), ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255,255))
                else:
                    if ArenaSide =='L':
                        cv2.putText(mask, str(instHN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                        cv2.putText(mask, str(instHN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])-POIrad/1.25, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                    elif ArenaSide == 'R':
                        cv2.putText(mask, str(instHN[ncirc])[0], ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255,255))
                        cv2.putText(mask, str(instHN[ncirc])[1], ( width-int(Xcoords[instN[ncirc]])+POIrad*0.06, int(Ycoords[instN[ncirc]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255,255))
        # Flip text so it lines up with prior image
        mask = cv2.flip(mask,1)
        background = background + mask
        if instdispN != 0:
            mask =  np.zeros((height,width,3), np.uint8)
            cv2.circle(mask, (int(dispx), int(dispy)), int(POIrad)/2, (0, 255, 255), 2)
            background = background + mask
        if instHN != 0:
            mask =  np.zeros((height,width,3), np.uint8)
            # Indicate which home plates will be targeted
            for ncirc in range(len(instHN)):
                cv2.circle(mask, (width-HXcoords[instHN[ncirc]], HYcoords[instHN[ncirc]]), int(POIrad)/3, (255, 0, 0), -1)
            # Add a zoomed in home plate for better visibility
            curcoord = range(2)
            hcoordsX = np.zeros((12, 8))
            hcoordsY = np.zeros((12, 8))
            for hrow in range(0,12):
                for hcol in range(0,8):
                    curcoord[0] = 350 - (hcol * 18)
                    curcoord[1] = float(425) - float(hrow * 18)
                    hcoordsX[hrow, hcol] = curcoord[0]
                    hcoordsY[hrow, hcol] = float(curcoord[1])
            zoomHX = np.reshape(hcoordsX, (12*8,1))
            zoomHY = np.reshape(hcoordsY, (12*8,1))
            cv2.rectangle(mask, (max(zoomHX)+15, max(zoomHY)+15), (min(zoomHX)-15,min(zoomHY)-15), (115, 115, 115), -1)
            # Also with bigger individual homes
            for ncirc in range(len(HXcoords)):
                cv2.circle(mask, (zoomHX[ncirc], zoomHY[ncirc]), int(POIrad/1.2), (-55, -55, -55), 1)
            if start == 'home':
                for ncirc in range(len(instHN)):
                    # Put string of fly in vector instN -> In the future: add left/right & homeToArn or the other way around
                    # Larger numbers have to be smaller
                    if instHN[ncirc] <= 9:
                        cv2.putText(mask, str(instHN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(255,0,0))
                    else:
                        cv2.putText(mask, str(instHN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(255,0,0))
            elif start =='arena':
                for ncirc in range(len(instN)):
                    if instHN[ncirc] <= 9:
                        cv2.putText(mask, str(instN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(255,0,0))
                    else:
                        cv2.putText(mask, str(instN[ncirc]), (max(zoomHX)*1.60 - int(zoomHX[instHN[ncirc]]-POIrad/1.25), int(zoomHY[instHN[ncirc]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(255,0,0))
            # Zoom-in lines
            cv2.line(mask, (max(zoomHX)+17, max(zoomHY)+10), (max(HXcoords)-10, max(HYcoords)+10), (35,35,35), thickness=1, lineType=1, shift=0)
            cv2.line(mask, (max(zoomHX)+17, max(zoomHY)-215), (max(HXcoords)-10, max(HYcoords)-100), (35,35,35), thickness=1, lineType=1, shift=0)
            mask = cv2.flip(mask,1)
            background = cv2.subtract(background, mask)
        # Flip entire image vertically so it lines up with reality
        background = cv2.flip(background,1)
        # Robot home text after flip
        cv2.putText(background, 'MAPLE', (width-43, 20),fontFace=1, fontScale=0.7, color=(0,0,0))
        cv2.putText(background, 'FlyPlate', (width-43, 30),fontFace=1, fontScale=0.7, color=(0,0,0))
        key = 0
        while key == 0:
            print 'Press ENTER or SPACE to accept.\nPress ESC to cancel instruction.'
            cv2.imshow("Instruction", background)
            key = cv2.waitKey(0)
            if key != 27 and key != 13 and key != 32:
                key = 0
            elif key == 27:
                cv2.destroyAllWindows()
                quit()
                return False
            elif key == 13 or 32:
                print 'Instruction accepted.'
                cv2.destroyAllWindows()
                return background

    # Visual update of current command
    def updateCurInstruct(self, background, N, HXcoords, HYcoords, Xcoords, Ycoords, dispx, dispy, instN, ArenaSide='L', instHN=0, instdispN=0, start='home'):
        update = background.copy()
        update = cv2.flip(update,1)
        if start =='home':
            if ArenaSide == 'L':
                p = (HXcoords[instHN[N]], HYcoords[instHN[N]])
                q = (Xcoords[instN[N]]+11, Ycoords[instN[N]])
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                angle = np.arctan2(p[1]-q[1], p[0]-q[0])
                p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
                int(q[1] + 9 * np.sin(angle + np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
                int(q[1] + 9 * np.sin(angle - np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
            elif ArenaSide == 'R':
                p = (HXcoords[instHN[N]], HYcoords[instHN[N]])
                q = (Xcoords[instN[N]]+1, Ycoords[instN[N]])
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                angle = np.arctan2(p[1]-q[1], p[0]-q[0])
                p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
                int(q[1] + 9 * np.sin(angle + np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
                int(q[1] + 9 * np.sin(angle - np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
        elif start =='arena':
            if ArenaSide == 'L':
                p = (Xcoords[instN[N]]+11, Ycoords[instN[N]])
                q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                angle = np.arctan2(p[1]-q[1], p[0]-q[0])
                p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
                int(q[1] + 9 * np.sin(angle + np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
                int(q[1] + 9 * np.sin(angle - np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
            elif ArenaSide == 'R':
                p = (Xcoords[instN[N]]+1, Ycoords[instN[N]])
                q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                angle = np.arctan2(p[1]-q[1], p[0]-q[0])
                p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
                int(q[1] + 9 * np.sin(angle + np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
                p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
                int(q[1] + 9 * np.sin(angle - np.pi/4)))
                cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
        elif start == 'dispenser':
            p = (int(dispx), int(dispy))
            q = (HXcoords[instHN[N]], HYcoords[instHN[N]])
            cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
            angle = np.arctan2(p[1]-q[1], p[0]-q[0])
            p = (int(q[0] + 9 * np.cos(angle + np.pi/4)),
            int(q[1] + 9 * np.sin(angle + np.pi/4)))
            cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
            p = (int(q[0] + 9 * np.cos(angle - np.pi/4)),
            int(q[1] + 9 * np.sin(angle - np.pi/4)))
            cv2.line(update, p, q, (0, 255, 255), 1, 1, 0)
        update = cv2.flip(update,1)
        cv2.imshow('Current instruction', update)
        cv2.waitKey(1)
        return {'background':background, 'update':update}

    # Permanently alters the background image and populates arena/home with successes
    def updatePastInstruct(self, background, N, HXcoords, HYcoords, Xcoords, Ycoords, POIrad, dispx, dispy, instN, Fail=0, instHN=0, instdispN=0, ArenaSide='L', start='home', updateWhich='home'):
        curcoord = range(2)
        hcoordsX = np.zeros((12, 8))
        hcoordsY = np.zeros((12, 8))
        for hrow in range(0,12):
            for hcol in range(0,8):
                curcoord[0] = 350 - (hcol * 18)
                curcoord[1] = float(425) - float(hrow * 18)
                hcoordsX[hrow, hcol] = curcoord[0]
                hcoordsY[hrow, hcol] = float(curcoord[1])
        zoomHX = np.reshape(hcoordsX, (12*8,1))
        zoomHY = np.reshape(hcoordsY, (12*8,1))
        width = max(HXcoords)*2
        background = cv2.flip(background,1)
        if updateWhich =='home':
            cv2.circle(background, (HXcoords[instHN[N]], HYcoords[instHN[N]]), int(POIrad)/3, (0,255-Fail*255,255*Fail), -1)
            # zoomed in versions
            if instHN[N] <= 9 and instdispN == 0:
                background = cv2.flip(background,1)
                cv2.putText(background, str(instHN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                background = cv2.flip(background,1)
            elif instHN[N] > 9 and instdispN == 0:
                background = cv2.flip(background,1)
                cv2.putText(background, str(instHN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.55, color=(0,255-Fail*255,255*Fail))
                background = cv2.flip(background,1)
            elif instdispN != 0:
                background = cv2.flip(background,1)
                cv2.putText(background, str(instdispN[N]), (max(zoomHX*1.6) - int(zoomHX[instHN[N]]-POIrad/1.25), int(zoomHY[instHN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                background = cv2.flip(background,1)
        elif updateWhich =='arena':
            if start == 'home':
                background = cv2.flip(background,1)
                if instHN[N] <= 9:
                    if ArenaSide == 'L':
                        cv2.putText(background, str(instHN[N]), (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                    elif ArenaSide =='R':
                        cv2.putText(background, str(instHN[N]), (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                else:
                    if ArenaSide == 'L':
                        cv2.putText(background, str(instHN[N])[0], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                        cv2.putText(background, str(instHN[N])[1], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                    elif ArenaSide =='R':
                        cv2.putText(background, str(instHN[N])[0], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                        cv2.putText(background, str(instHN[N])[1], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
            elif start == 'arena':
                background = cv2.flip(background,1)
                if instN[N] <= 9:
                    if ArenaSide == 'L':
                        cv2.putText(background, str(instN[N]), (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                    elif ArenaSide =='R':
                        cv2.putText(background, str(instN[N]), (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/3) ),fontFace=1, fontScale=0.7, color=(0,255-Fail*255,255*Fail))
                else:
                    if ArenaSide == 'L':
                        cv2.putText(background, str(instN[N])[0], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                        cv2.putText(background, str(instN[N])[1], (int(width-Xcoords[instN[N]]-POIrad/1.25), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                    elif ArenaSide =='R':
                        cv2.putText(background, str(instN[N])[0], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]-POIrad/16) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
                        cv2.putText(background, str(instN[N])[1], (int(width-Xcoords[instN[N]]+POIrad*0.10), int(Ycoords[instN[N]]+POIrad/1.4) ),fontFace=1, fontScale=0.60, color=(0,255-Fail*255,255*Fail))
            else:
                background = cv2.flip(background,1)
            background = cv2.flip(background,1)
        background = cv2.flip(background,1)
        cv2.destroyAllWindows()
        cv2.imshow('Current instruction', background)
        cv2.waitKey(1)
        return background
