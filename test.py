import imgprocess

reference = (390, 410)
PPMM = 28
padLocation = (1000, 3000, 0, 0, 0)     #location of top left hand corner of pad
padSize = (500, 500)                    #size of the pad in x, y
mazeLocation = (3000, 3000, 0, 0, 0)
imageSize = (1900/PPMM, 1900/PPMM) #(1900, 1900) in pixels

def getAllFlies(padLocation, padSize, imageSize, mazeLocation):
    
    duration = 2
    plateBool = 1
    a = imageProcess()
    
    #how many images do we need to take?
    xSweeps = padSize[0]/imageSize[0] + 1
    ySweeps = padSize[1]/imageSize[1] + 1
    
    #all the different locations we will need to image
    camXY = []
    for x, y in range(xSweeps+1), range(ySweeps+1):
        camXY.append((x * padSize/xSweeps, y * padSize/ySweeps))
    
    for (x,y) in camXY:

        print "Imaging:", (x,y)
        s, img = close.read()
        targets = a.execute(img, reference)

        for (x,y) in targets:
            pt = (reference[0] + x, reference[1] + y, 0, 0, 0)

getAllFlies(padLocation, padSize, imageSize, mazeLocation)   