# Highest order command to withdraw a single fly from a single well in the housing module
def homeWithdraw(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, dislodgeZ=10, vacBurst=1, vacDur=4000, homeZ=45):
    if refptX != 'N':
        self.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
        self.dwell(t=1)
    self.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, dislodgeZ, 5000])        # Go to actual home
    self.dwell(t=1)
    self.flyManipAir(True)
    trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
    if trylowerHome['limit'] == 0:
        self.dwell(t=1)
        for b in range(0,vacBurst):
        	self.flyManipAir(False)
            self.smallPartManipVac(True)
            self.dwell(t=rand.choice(range(2,4)))
            self.smallPartManipVac(False)
            self.dwell(t=rand.choice(range(2,4)))
        self.smallPartManipVac(True)
        self.dwell(t=vacDur)
        self.moveRel(pt=[0, 0, 0, 0, -homeZ])
        self.dwell(t=10)
    else:
    	self.flyManipAir(False)
    	self.home()
    return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

# Highest order command to deposit a single fly in a well in the housing module
def homeDeposit(self, homecoordX, homecoordY, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44):
	if refptX != 'N':
        self.moveToSpd(pt=[float(refptX), float(refptY), 0, 0, 10, 5000])       # Allgin to outermost flyhome to prevent tripping
        self.dwell(t=1)
    self.moveToSpd(pt=[float(homecoordX), float(homecoordY), 0, 0, 10, 5000])        # Go to actual home
    self.dwell(t=1)
    trylowerHome = self.lowerCare(z=homeZ, descendZ=carefulZ, retreatZ=carefulZ)      # Move into home - check Z height!
    if trylowerHome['limit'] == 0:
        self.dwell(t=1)
        self.smallPartManipVac(False)
        for b in range(0,vacBurst):
            self.flyManipAir(True)
            self.dwell(t=rand.choice(range(5,6)))
            self.flyManipAir(False)
            self.dwell(t=rand.choice(range(5,6)))
        self.dwell(t=50)
        self.moveRel(pt=[0, 0, 0, 0, -homeZ])
        self.dwell(t=10)
    else:
    	self.home()
    return {'homeX': homecoordX, 'homeY': homecoordY, 'limit': trylowerHome['limit']}

# Highest order command to withdraw a single fly from a single arena in the behavioral module (Different withdraw-strategies accessible using vacstrategy)
def arenaWithdraw(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, vacPos, vacZ, closePos, vacstrategy=2, vacBurst=1, imgshow=0):
	strategy = vacstrategy
	missonce = 0
	print 'Using strategy', strategy
    self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
    self.dwell(t=1)
    degs1 = int(self.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
    self.dwell(t=5)
    self.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
    self.dwell(t=10)
    Mid1 = self.getCurrentPosition()
    self.dwell(1)
    endpos1 = vacPos
    if strategy == 3 or strategy == 6:
    	self.smallPartManipVac(True)
    tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
    miss = tempCoord['limit']
    missonce = missonce + tempCoord['limitonce']
    self.dwell(50)
    if miss == 1:
        print 'Possible misalignment - resetting...'
        self.home()
        return {'miss': miss, 'missonce': missonce}
    elif miss != 1:
        self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        if strategy == 1:
            self.smallPartManipVac(False)
            for b in range(0,vacBurst):
                self.smallPartManipVac(True)
                self.dwell(t=200)
                self.smallPartManipVac(False)
                self.dwell(t=10)
            self.smallPartManipVac(True)
            self.dwell(t=400)
            self.smallPartManipVac(False)
            self.dwell(t=5)
            self.smallPartManipVac(True)
        elif strategy == 2:
        	self.smallPartManipVac(False)
            self.flyManipAir(True)
            self.dwell(t=5)
            self.flyManipAir(False)
            self.dwell(t=5)
            self.flyManipAir(True)
            self.dwell(t=5)
            self.flyManipAir(False)
            self.dwell(t=5)
            self.smallPartManipVac(True)
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ+1])
            self.dwell(t=50)
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ-1])
            self.dwell(t=50)
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            self.dwell(t=50)
        elif strategy == 3:
        	sweeppos1 = tempCoord['endDeg']
        	if sweeppos1 < 180:
        		sweeppos2 = 140
        	else:
        		sweeppos2 = 220
        	checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
        	missonce = missonce + checkformiss['limitonce']
        	self.dwell(50)
        	checkformiss = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
        	missonce = missonce + checkformiss['limitonce']
        elif strategy == 4:
            self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
            self.dwell(t=10)
            self.smallPartManipVac(True)
            self.dwell(t=20000)
        elif strategy == 5:
        	self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        	self.dwell(t=5)
        	self.flyManipAir(True)
        	self.dwell(t=500)
        	self.flyManipAir(False)
        	self.smallPartManipVac(True)
        	self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,vacZ])
        	self.dwell(t=1000)
        elif strategy == 6:
        	sweeppos1 = tempCoord['endDeg']
        	if sweeppos1 < 180:
        		sweeppos2 = 140
        	else:
        		sweeppos2 = 220
        	self.flyManipAir(True)
        	self.dwell(t=5)
        	self.flyManipAir(False)
        	self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos1, endpos=sweeppos2, spd=2000, descendZ=0)
        	self.flyManipAir(True)
        	self.dwell(t=5)
        	self.flyManipAir(False)
        	self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = vacZ, startpos=sweeppos2, endpos=sweeppos1, spd=2000, descendZ=0)
        	self.dwell(50)
        endpos2 = closePos
        tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
        missonce = missonce + tempCoord['limitonce']
        endpos1 = tempCoord['endDeg']
        self.dwell(10)
        self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
        self.dwell(10)
        return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

# Highest order command to deposit a single fly in a single arena in the behavioral module
def arenaDeposit(self, camcoordX, camcoordY, camcoordZ, arenacoordX, arenacoordY, arenaRad, turnZ, airPos, airZ, closePos, airBurst=1, imgshow=0):
	missonce = 0
	self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10, 5000])
    self.dwell(t=1)
    degs1 = int(self.findDegs(slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0))
    self.dwell(t=5)
    self.moveToSpd(pt=[float(arenacoordX), float(arenacoordY), 0, camcoordZ, 10, 5000])
    self.dwell(t=10)
    Mid1 = self.getCurrentPosition()
    self.dwell(1)
    endpos1 = airPos
    tempCoord = self.tryOpening(mid = [Mid1[0],Mid1[1]], r = float(arenaRad), z = turnZ, startpos=degs1, endpos=endpos1, spd=2000, descendZ=5)
    miss = tempCoord['limit']
    missonce = missonce + tempCoord['limitonce']
    self.dwell(50)
    if miss == 1:
        print 'Possible misalignment - full reset...'
        self.home()
        return {'miss': miss, 'missonce': missonce}
    elif miss != 1:
        self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,airZ])
        self.smallPartManipVac(False)
        self.dwell(t=5)
        for b in range(0,airBurst):
            self.flyManipAir(True)
            self.dwell(t=rand.choice(range(5,6)))
            self.flyManipAir(False)
            self.dwell(t=rand.choice(range(5,6)))
        self.dwell(t=50)
        endpos2 = closePos
        tempCoord = self.tryOpening(mid = tempCoord['oldMid'], r = float(arenaRad), z = turnZ, startpos=tempCoord['endDeg'], endpos=endpos2, spd=2000, descendZ=0)
        missonce = missonce + tempCoord['limitonce']
        endpos1 = tempCoord['endDeg']
        self.dwell(50)
        self.moveZ([tempCoord['endXY'][0],tempCoord['endXY'][1],0,0,10])
        self.dwell(10)
        return {'miss':miss, 'arenaX':arenacoordX, 'arenaY':arenacoordY, 'endX': tempCoord['endXY'][0], 'endY':tempCoord['endXY'][1], 'endpos':endpos1, 'missonce':missonce}

# Moves robot to dispenser location, sends command to dispend fly, tries dispiter times to vacuum fly out
def dispenseWithdraw(self, dispX, dispY, dispZ, dispiter=2, onlyifsure=1):
	dispsuccess = 0
	self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
	self.dwell(50)
	limit = self.lowerCare(dispZ, descendZ=6, retreatZ=12)
	if onlyifsure == 0:
		print 'Depositing even if fly may be stuck.'
	if limit != 1:
    	self.smallPartManipVac(True)
    	for iter in range(dispiter):
    		dispsuccess = self.dispenseFly()
    		if dispsuccess == 1:
    			print 'Fly dispensed.'
    			self.dwell(2000)
    			self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    			return dispsuccess
    		elif dispsuccess == 0:
    			self.smallPartManipVac(False)
    		elif dispsuccess == 2 and onlyifsure == 1:
    			self.smallPartManipVac(False)
    		elif dispsuccess == 2 and onlyifsure == 0:
    			print 'Quantum fly in tunnel - better safe than sorry protocol commencing.'
    			self.dwell(3000)
    			self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
    elif limit == 1:
    	print 'Possible misallignment. Check fly dispenser.'
    	self.home()
    	return None
    self.moveToSpd(pt=[float(dispX), float(dispY), 0, 0, 10, 5000])
	return dispsuccess
