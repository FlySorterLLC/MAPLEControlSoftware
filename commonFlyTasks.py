


# Highest order command to withdraw a single fly from a single well in the housing module
def homeWithdraw(robot, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, dislodgeZ=10, vacBurst=1, vacDur=4000, homeZ=45):
    if refptX != 'N':
        robot.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
        robot.dwell(t=1)
    robot.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, dislodgeZ, 5000])        # Go to actual home
    robot.dwell(t=1)
    robot.flyManipAir(True)
    trylowerHome = robot.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
    if trylowerHome['limit'] == 0:
        robot.dwell(t=1)
        for b in range(0,vacBurst):
        	robot.flyManipAir(False)
            robot.smallPartManipVac(True)
            robot.dwell(t=rand.choice(range(2,4)))
            robot.smallPartManipVac(False)
            robot.dwell(t=rand.choice(range(2,4)))
        robot.smallPartManipVac(True)
        robot.dwell(t=vacDur)
        robot.moveRel(pt=[0, 0, 0, 0, -homeZ])
        robot.dwell(t=10)
    else:
    	robot.flyManipAir(False)
    	robot.home()
    return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

# Highest order command to deposit a single fly in a well in the housing module
def homeDeposit(robot, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44):
	if refptX != 'N':
        robot.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
        robot.dwell(t=1)
    robot.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, 10, 5000])        # Go to actual home
    robot.dwell(t=1)
    trylowerHome = robot.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
    if trylowerHome['limit'] == 0:
        robot.dwell(t=1)
        robot.smallPartManipVac(False)
        for b in range(0,vacBurst):
            robot.flyManipAir(True)
            robot.dwell(t=rand.choice(range(5,6)))
            robot.flyManipAir(False)
            robot.dwell(t=rand.choice(range(5,6)))
        robot.dwell(t=50)
        robot.moveRel(pt=[0, 0, 0, 0, -homeZ])
        robot.dwell(t=10)
    else:
    	robot.home()
    return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

# Highest order command to withdraw a single fly from a single arena in the behavioral module (Different withdraw-strategies accessible using vacstrategy)
def arenaWithdraw(robot, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, vacPos, vacZ, closePos, vacstrategy=2, vacBurst=1, imgshow=0):
	strategy = vacstrategy
	missonce = 0
	print 'Using strategy', strategy
    robot.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
    robot.dwell(t=1)
    degs1 = int(robot.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
    robot.dwell(t=5)
    robot.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
    robot.dwell(t=10)
    Mid1 = robot.getCurrentPosition()
    robot.dwell(1)
    endpos1 = vacPos
    if strategy == 3 or strategy == 6:
    	robot.smallPartManipVac(True)
    tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
    miss = tempCoord['limit']
    missonce = missonce + tempCoord['limitonce']
    robot.dwell(50)
    if miss == 1:
        print 'Possible misalignment - resetting...'
        robot.home()
        return {'miss': miss, 'missonce': missonce}
    elif miss != 1:
        robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        if strategy == 1:
            robot.smallPartManipVac(False)
            for b in range(0,vacBurst):
                robot.smallPartManipVac(True)
                robot.dwell(t=200)
                robot.smallPartManipVac(False)
                robot.dwell(t=10)
            robot.smallPartManipVac(True)
            robot.dwell(t=400)
            robot.smallPartManipVac(False)
            robot.dwell(t=5)
            robot.smallPartManipVac(True)
        elif strategy == 2:
        	robot.smallPartManipVac(False)
            robot.flyManipAir(True)
            robot.dwell(t=5)
            robot.flyManipAir(False)
            robot.dwell(t=5)
            robot.flyManipAir(True)
            robot.dwell(t=5)
            robot.flyManipAir(False)
            robot.dwell(t=5)
            robot.smallPartManipVac(True)
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ+1])
            robot.dwell(t=50)
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ-1])
            robot.dwell(t=50)
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            robot.dwell(t=50)
        elif strategy == 3:
        	sweeppos1 = tempCoord['endDeg']
        	if sweeppos1 < 180:
        		sweeppos2 = 140
        	else:
        		sweeppos2 = 220
        	checkformiss = robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
        	missonce = missonce + checkformiss['limitonce']
        	robot.dwell(50)
        	checkformiss = robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
        	missonce = missonce + checkformiss['limitonce']
        elif strategy == 4:
            robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            robot.dwell(t=10)
            robot.smallPartManipVac(True)
            robot.dwell(t=20000)
        elif strategy == 5:
        	robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        	robot.dwell(t=5)
        	robot.flyManipAir(True)
        	robot.dwell(t=500)
        	robot.flyManipAir(False)
        	robot.smallPartManipVac(True)
        	robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        	robot.dwell(t=1000)
        elif strategy == 6:
        	sweeppos1 = tempCoord['endDeg']
        	if sweeppos1 < 180:
        		sweeppos2 = 140
        	else:
        		sweeppos2 = 220
        	robot.flyManipAir(True)
        	robot.dwell(t=5)
        	robot.flyManipAir(False)
        	robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
        	robot.flyManipAir(True)
        	robot.dwell(t=5)
        	robot.flyManipAir(False)
        	robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
        	robot.dwell(50)
        endpos2 = closePos
        tempCoord = robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
        missonce = missonce + tempCoord['limitonce']
        endpos1 = tempCoord['endDeg']
        robot.dwell(10)
        robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
        robot.dwell(10)
        return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

# Highest order command to deposit a single fly in a single arena in the behavioral module
def arenaDeposit(robot, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, airPos, airZ, closePos, airBurst=1, imgshow=0):
	missonce = 0
	robot.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
    robot.dwell(t=1)
    degs1 = int(robot.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
    robot.dwell(t=5)
    robot.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
    robot.dwell(t=10)
    Mid1 = robot.getCurrentPosition()
    robot.dwell(1)
    endpos1 = airPos
    tempCoord = robot.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
    miss = tempCoord['limit']
    missonce = missonce + tempCoord['limitonce']
    robot.dwell(50)
    if miss == 1:
        print 'Possible misalignment - full reset...'
        robot.home()
        return {'miss': miss, 'missonce': missonce}
    elif miss != 1:
        robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,airZ])
        robot.smallPartManipVac(False)
        robot.dwell(t=5)
        for b in range(0,airBurst):
            robot.flyManipAir(True)
            robot.dwell(t=rand.choice(range(5,6)))
            robot.flyManipAir(False)
            robot.dwell(t=rand.choice(range(5,6)))
        robot.dwell(t=50)
        endpos2 = closePos
        tempCoord = robot.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
        missonce = missonce + tempCoord['limitonce']
        endpos1 = tempCoord['endDeg']
        robot.dwell(50)
        robot.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
        robot.dwell(10)
        return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

# Moves robot to dispenser location, sends command to dispend fly, tries dispiter times to vacuum fly out
def dispenseWithdraw(robot, dispenser, dispiter=2, onlyifsure=1):
	dispsuccess = 0
    dispX, dispY, dispZ = dispenser.dispenserPoint
	robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
	robot.dwell(50)
	limit = robot.lowerCare(dispZ, descendZ=6, retreatZ=12)
	if onlyifsure == 0:
		print 'Depositing even if fly may be stuck.'
	if limit != 1:
    	robot.smallPartManipVac(True)
    	for iter in range(dispiter):
    		dispsuccess = dispenser.dispenseFly()
    		if dispsuccess == 1:
    			print 'Fly dispensed.'
    			robot.dwell(2000)
    			robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    			return dispsuccess
    		elif dispsuccess == 0:
    			robot.smallPartManipVac(False)
    		elif dispsuccess == 2 and onlyifsure == 1:
    			robot.smallPartManipVac(False)
    		elif dispsuccess == 2 and onlyifsure == 0:
    			print 'Quantum fly in tunnel - better safe than sorry protocol commencing.'
    			robot.dwell(3000)
    			robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    elif limit == 1:
    	print 'Possible misallignment. Check fly dispenser.'
    	robot.home()
    	return None
    robot.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
	return dispsuccess

# Tries to dispense and deposit all newly hatched flies into home plate / saves how many flies were successfully dispensed for longer processing
def dispenseHIfHatched(robot, homecoordX, homecoordY, dispenser, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, maxconsecstuck=4):
    nDispensed = carryovernDispensed
    IsHatched = 1
    consecStuck = 0
    while IsHatched == 1 or (IsHatched == 2 and onlyifsure == 0) and not consecStuck > maxconsecstuck:
        IsHatched = robot.dispenseWithdraw(dispenser, dispiter=dispiter, onlyifsure=onlyifsure)
        if IsHatched == 1:
            robot.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
            nDispensed = nDispensed+1
        elif IsHatched == 2 and onlyifsure == 0:
            robot.homeDeposit(homecoordX=homecoordX[nDispensed], homecoordY=homecoordY[nDispensed], refptX='N', refptY='N', carefulZ=carefulZ, vacBurst=vacBurst, homeZ=homeZ)
            nDispensed = nDispensed +1
            consecStuck = consecStuck +1
        else:
            break
            print 'No fly dispensed.'
    print 'Dispensed', nDispensed, 'flies total.'
    return nDispensed

# Repeatedly collect successfully dispensed flies (newly hatched) and deposit into consecutive single wells in the housing module
# Not accurate loop time-wise but ensures a minimum amount of time tried collecting. All times in ms.
def collectHatchedForT(robot, homecoordX, homecoordY, dispenser, onlyifsure=1, carefulZ=9, vacBurst=1, homeZ=44, dispiter=3, carryovernDispensed=0, collectT=3600, collectInt=60, maxconsecstuck=4):
    for n in range(collectT/collectInt):
        carryovernDispensed = dispenseHIfHatched(robot, homecoordX=homecoordX, homecoordY=homecoordY, dispenser, onlyifsure=onlyifsure, carefulZ=9, vacBurst=vacBurst, homeZ=homeZ, dispiter=dispiter, carryovernDispensed=carryovernDispensed, maxconsecstuck=maxconsecstuck)
        time.sleep(collectInt)
