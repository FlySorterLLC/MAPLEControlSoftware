#! /usr/bin/env python

# Remote listening script for MAPLE robot
#
# Waits for email to arrive signaling action to take
# Keeps user informed of errors, progress.

import traceback
from email import parser
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
import smtplib
import email
import poplib
import urllib2


    # Sends email containing images of arenanum arenas
    def notifyUserFail(self, arenanum, mailfrom, attPic=0, qualPic=25, attImg=1, delFiles=1, robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
        gmail_user = robotEMailAccount
        gmail_password = PWrobotEMailAccount
        try:
            server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
        except:
            print 'SMTP went wrong...'
        try:
            server.ehlo()
        except:
            print 'Handshake went wrong...'
        try:
            server.login(gmail_user, gmail_password)
        except:
            print 'Login went wrong...'
        if attImg == 1:
            msg = MIMEMultipart()
            for imgnum in range(len(arenanum)):
                curarenanum = str(arenanum[imgnum])
                imgname = curarenanum + 'errImage.png'
                with open(imgname, 'rb') as fp:
                    img = MIMEImage(fp.read())
                msg.attach(img)
                if delFiles == 1:
                	os.remove(imgname)
            fp.close()
            arenanum = str(arenanum)
            msg.preamble = 'Arena ' + arenanum + ' failed to unload.'
            msg['Subject'] = 'Failure: Arenas ' + arenanum + ' Withdraw'
        if attPic == 1:
            arenanum = str(arenanum)
            self.light(True)
            time.sleep(0.2)
            self.cam.start_live()
            self.cam.snap_image()
            self.cam.save_image(arenanum + 'errImage.png', 1, jpeq_quality=qualPic)
            self.cam.stop_live()
            self.dwell(50)
            self.light(False)
            msg['Subject'] = 'Failure: Arena ' + arenanum + ' Withdraw'
            msg = MIMEMultipart()
            msg.preamble = 'Arena ' + arenanum + ' failed to unload.'
            fp = open(arenanum + 'errImage.png', 'rb')
            img = MIMEImage(fp.read())
            fp.close()
            msg.attach(img)
            if delFiles == 1:
            	os.remove(arenanum + 'errImage.png')
        if attPic == 0 and attImg == 0:
            arenanum = str(arenanum)
            msg['Subject'] = 'Failure: Arena ' + arenanum + ' Withdraw'
        msg['From'] = gmail_user
        msg['To'] = gmail_user
        server.sendmail(gmail_user, mailfrom, msg.as_string())
        server.quit()
        return

    # Reads the most recent emails at associated gmail account and parses instructions (Incorrect instruction format will not cause error-state). Note: formatted for gmail and iOS Mail application.
    def receiveMailInstruct(self, delMail=1, subjKeyVect=['INSTRUCT', 'A2H', 'H2A', 'SWP', 'SWPFL', 'HELP', 'CLCT'], robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):		# Remeber to add all programmed keywords!
	    try:
	        pop_conn = poplib.POP3_SSL('pop.gmail.com')
	        pop_conn.user(robotEMailAccount)
	        pop_conn.pass_(PWrobotEMailAccount)
	        messages = [pop_conn.retr(i) for i in reversed(range(1, len(pop_conn.list()[1]) + 1))]
	        messages = ["\n".join(mssg[1]) for mssg in messages]
	        messages = [parser.Parser().parsestr(mssg) for mssg in messages]
	        for i, message in enumerate(messages):
	            if delMail==1:
	                pop_conn.dele(i+1)
	            if message['subject'] == 'HELP':
	            	return {'values': range(0,2), 'instruct': message['subject'], 'from': message['from']}
	            if message['subject'] in subjKeyVect:
	                print 'Instruction keyword found.'
	                message.set_type('text/plain')
	                try:		# works on gmail account from PC
	                    prefilter = str(message.get_payload(0))
	                    prefilter = prefilter.split('\n')[3].split(',')
	                except:		# works from iphone
	                	prefilter = str(message).split('"us-ascii"')[1].split('\n')[2].split(',')
	                postfilter = range(len(prefilter))
	                for i in range(len(prefilter)):
	                	postfilter[i] = int(prefilter[i])
	                pop_conn.quit()
	                return {'values': postfilter, 'instruct': message['subject'], 'from': message['from']}
	        pop_conn.quit()
	        return
	    except:
	        return None

	# Puts robot in listen mode (Repeated receiveMailInstruct) and updates listening state in online monitor
    def listenMode(self, duration=60, listenInterval=10):		# in seconds
        print 'Listening for', duration, 'seconds in', listenInterval, 'second intervals.'
        # Puts monitor to mode 2 for listening
        try:
            urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=2')
        except:
            print 'Could not reach monitor URL'
        for interval in range(duration/listenInterval):
            time.sleep(listenInterval)
            mail = self.receiveMailInstruct()
            if mail != None:
                return mail
        print 'No instructions received.'
        # Sends 0 to monitor after listening mode is over
        try:
            urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=0')
        except:
            print 'Could not reach monitor URL.'
        return mail

    # Translates correct email commands into preprogrammed robot routines (Incorrect values will cause error-state requiring manual robot reset)
    def doInstruct(self, mailfrom, instruction, values, CamX, CamY, CamZ, ManipX, ManipY, arenaRad, HomeX, HomeY, HomeZwd, HomeZdp, turnZ, vacZ, dispX=639.5, dispY=113, dispZ=34, disponlyifsure=0, maxconsecstuck=6,robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
        if instruction == 'SWP':
            flyremainvect = self.sweep(CamX[values[0]:values[1]], CamY[values[0]:values[1]], camz=CamZ)
            unsurevect = self.sweep(CamX[flyremainvect], CamY[flyremainvect], camz=CamZ)
            try:
                flyremainvect = flyremainvect[unsurevect]
                self.SaveArenaPic(Xcoords=CamX[flyremainvect], Ycoords=CamY[flyremainvect], IndVect=flyremainvect)
                self.notifyUserFail(flyremainvect, mailfrom=mailfrom, attImg=1)
            except:
                print 'No failed arenas detected'
        elif instruction == 'SWPFL':
            self.SaveArenaPic(Xcoords=CamX[values[0]:values[1]], Ycoords=CamY[values[0]:values[1]], IndVect=range(values[0], values[1]))
            self.notifyUserFail(range(values[0], values[1]), mailfrom=mailfrom, attImg=1)
        elif instruction == 'H2A':
            try:		# If not specified, add repetitions as 1
                values[4]
            except:
                values.append(1)
            for repeat in range(0, values[4]):
                for depositValue in range(values[0], values[1]):
                    self.homeWithdraw(homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=HomeZwd)
                    checkmiss = self.arenaDeposit(camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, airPos=values[2], airZ=vacZ, closePos=values[3])
                    if (values[1] - depositValue)%4 == 0:
                        print 'Resetting Z position to maintain accuracy.'
                        self.homeZ()
                    if checkmiss['missonce'] != 0:
                        print 'Missed opening at least once - realigning...'
                        self.homeZ()
        elif instruction == 'A2H':
            try:		# If not specified, add repetitions as 1
                values[5]
            except:
                values.append(1)
            for repeat in range(0, values[5]):
                for depositValue in range(values[0], values[1]):
                    checkmiss = self.arenaWithdraw(camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, vacPos=values[2], vacZ=vacZ, closePos=values[3], vacstrategy=values[4])
                    self.homeDeposit(homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=HomeZdp)
                    if (values[1] - depositValue)%4 == 0:
                        print 'Resetting Z position to maintain accuracy.'
                        self.homeZ()
                    if checkmiss['missonce'] != 0:
                        print 'Missed opening at least once - realigning...'
                        self.homeZ()
        elif instruction == 'HELP':
            gmail_user = robotEMailAccount
            gmail_password = PWrobotEMailAccount
            try:
               server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
            except:
                print 'SMTP went wrong...'
            try:
                server.ehlo()
            except:
                print 'Handshake went wrong...'
            try:
                server.login(gmail_user, gmail_password)
            except:
                print 'Login went wrong...'
            msg = MIMEText('Format: Use KEYWORD in subject line. Enter arenas and settings as "#,#,#,..." without spaces.\n \nKEYWORDS: \nSWP: Sweeps from arenas [1] through [2] and sends back pictures of arenas in which motion was detected.\nSWPFL: Sends back pictures of arenas [1] through [2].\nA2H: Moves flies in arenas [1] through [2] to their homeplates. Vacuums at radian [3] and closes the lid at radian [4] using strategy [5].\nH2A: Same as A2H but from homeplates to arenas.\nCLCT: Starts virgin collection process for [1] seconds total every [2] seconds.\n \nExample:\nSubject: A2H\nText: 0,18,50,180,2.')
            msg['Subject'] = 'RE: SantaFe Remote Access Help'
            msg['From'] = robotEMailAccount
            msg['To'] = mailfrom
            server.sendmail(robotEMailAccount, mailfrom, msg.as_string())
            server.quit()
        elif instruction == 'CLCT':
            self.collectHatchedForT(homecoordX=HomeX, homecoordY=HomeY, dispX=dispX, dispY=dispY, dispZ=dispZ, onlyifsure=disponlyifsure, carefulZ=9, vacBurst=2, homeZ=44, dispiter=1, carryovernDispensed=0, maxconsecstuck=maxconsecstuck, collectT=values[0], collectInt=values[1])


    # Captures arena picture at location (Images named consecutively if multiple coordinates specified)
    def SaveArenaPic(self, Xcoords, Ycoords, IndVect, qualPic=25, Zcam=40, ImgName='errImage.png'):
        self.light(True)
        self.cam.start_live()
        for ImgNum in range(len(Xcoords)):
            self.moveToSpd(pt=[float(Xcoords[ImgNum]), float(Ycoords[ImgNum]), 0, Zcam, 10, 5000])
            self.dwell(50)		# Put higher to reduce effect of motion-caused rig trembling on picture
            self.cam.snap_image()
            curInd = str(IndVect[ImgNum])
            self.cam.save_image(curInd + 'errImage.png', 1, jpeq_quality=qualPic)
            self.dwell(10)
        self.cam.stop_live()
        self.light(False)


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
		mask = 	np.zeros((height,width,3), np.uint8)
		if start == 'arena':		# Considers fly in arena the 'actual' ID
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
		elif start == 'home':		# Considers fly in home to be the start ID
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
			mask = 	np.zeros((height,width,3), np.uint8)
			cv2.circle(mask, (int(dispx), int(dispy)), int(POIrad)/2, (0, 255, 255), 2)
			background = background + mask
		if instHN != 0:
			mask = 	np.zeros((height,width,3), np.uint8)
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
		cv2.putText(background, 'Robot', (width-43, 20),fontFace=1, fontScale=0.7, color=(0,0,0))
		cv2.putText(background, 'Home', (width-43, 30),fontFace=1, fontScale=0.7, color=(0,0,0))
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
