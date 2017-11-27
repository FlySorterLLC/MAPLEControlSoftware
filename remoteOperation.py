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
import commonFlyTasks

# Sends email containing images of arenanum arenas
def notifyUserFail(robot, arenanum, mailfrom, attPic=0, qualPic=25, attImg=1, delFiles=1, robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
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
        robot.light(True)
        time.sleep(0.2)
        robot.cam.start_live()
        robot.cam.snap_image()
        robot.cam.save_image(arenanum + 'errImage.png', 1, jpeq_quality=qualPic)
        robot.cam.stop_live()
        robot.dwell(50)
        robot.light(False)
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
def receiveMailInstruct(delMail=1, subjKeyVect=['INSTRUCT', 'A2H', 'H2A', 'SWP', 'SWPFL', 'HELP', 'CLCT'], robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):		# Remeber to add all programmed keywords!
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
def listenMode(duration=60, listenInterval=10):		# in seconds
    print 'Listening for', duration, 'seconds in', listenInterval, 'second intervals.'
    # Puts monitor to mode 2 for listening
    try:
        urllib2.urlopen('http://lab.debivort.org/mu.php?id=santaFe&st=2')
    except:
        print 'Could not reach monitor URL'
    for interval in range(duration/listenInterval):
        time.sleep(listenInterval)
        mail = receiveMailInstruct()
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
def doInstruct(robot, dispenser, mailfrom, instruction, values, CamX, CamY, CamZ, ManipX, ManipY, arenaRad, HomeX, HomeY, HomeZwd, HomeZdp, turnZ, vacZ, dispX=639.5, dispY=113, dispZ=34, disponlyifsure=0, maxconsecstuck=6,robotEMailAccount='example@gmail.com', PWrobotEMailAccount='examplePW'):
    if instruction == 'SWP':
        flyremainvect = robot.sweep(CamX[values[0]:values[1]], CamY[values[0]:values[1]], camz=CamZ)
        unsurevect = robot.sweep(CamX[flyremainvect], CamY[flyremainvect], camz=CamZ)
        try:
            flyremainvect = flyremainvect[unsurevect]
            robot.SavePicAt(Xcoords=CamX[flyremainvect], Ycoords=CamY[flyremainvect], IndVect=flyremainvect)
            notifyUserFail(flyremainvect, mailfrom=mailfrom, attImg=1)
        except:
            print 'No failed arenas detected'
    elif instruction == 'SWPFL':
        robot.SavePicAt(Xcoords=CamX[values[0]:values[1]], Ycoords=CamY[values[0]:values[1]], IndVect=range(values[0], values[1]))
        notifyUserFail(range(values[0], values[1]), mailfrom=mailfrom, attImg=1)
    elif instruction == 'H2A':
        try:		# If not specified, add repetitions as 1
            values[4]
        except:
            values.append(1)
        for repeat in range(0, values[4]):
            for depositValue in range(values[0], values[1]):
                commonFlyTasks.homeWithdraw(homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1, homeZ=HomeZwd)
                checkmiss = commonFlyTasks.arenaDeposit(camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, airPos=values[2], airZ=vacZ, closePos=values[3])
                if (values[1] - depositValue)%4 == 0:
                    print 'Resetting Z position to maintain accuracy.'
                    robot.homeZ2()
                if checkmiss['missonce'] != 0:
                    print 'Missed opening at least once - realigning...'
                    robot.homeZ2()
    elif instruction == 'A2H':
        try:		# If not specified, add repetitions as 1
            values[5]
        except:
            values.append(1)
        for repeat in range(0, values[5]):
            for depositValue in range(values[0], values[1]):
                checkmiss = commonFlyTasks.arenaWithdraw(robot, camcoordX=CamX[depositValue], camcoordY=CamY[depositValue], camcoordZ=CamZ, arenacoordX=ManipX[depositValue], arenacoordY=ManipY[depositValue], arenaRad=arenaRad, turnZ=turnZ, vacPos=values[2], vacZ=vacZ, closePos=values[3], vacstrategy=values[4])
                commonFlyTasks.homeDeposit(robot, homecoordX=HomeX[depositValue], homecoordY=HomeY[depositValue], refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=HomeZdp)
                if (values[1] - depositValue)%4 == 0:
                    print 'Resetting Z position to maintain accuracy.'
                    robot.homeZ2()
                if checkmiss['missonce'] != 0:
                    print 'Missed opening at least once - realigning...'
                    robot.homeZ2()
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
        msg['Subject'] = 'RE: MAPLE Remote Access Help'
        msg['From'] = robotEMailAccount
        msg['To'] = mailfrom
        server.sendmail(robotEMailAccount, mailfrom, msg.as_string())
        server.quit()
    elif instruction == 'CLCT':
        commonFlyTasks.collectHatchedForT(robot, HomeX, HomeY, dispenser, onlyifsure=disponlyifsure, carefulZ=9, vacBurst=2, homeZ=44, dispiter=1, carryovernDispensed=0, maxconsecstuck=maxconsecstuck, collectT=values[0], collectInt=values[1])
