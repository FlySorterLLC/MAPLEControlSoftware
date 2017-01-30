## Genetic algorithm to find best performing parameter values



import numpy as np
import pyicic.IC_ImagingControl
import math
import cv2
import random as rand
import matplotlib.pyplot as plt


popsize = 100

genes = np.array([[range(0,255)], [range(0,255)], [range(0,255)], [range(0,255)], [range(1,50)]])		# change these accordingly; gene4 needs decimals but thats done in findOpening

geneamount = len(genes)

popgenes = np.zeros((popsize, geneamount))

fitness = np.ones((popsize, 1))

population = np.append([popgenes], [fitness], axis=2)

newpop = np.append([popgenes], [fitness], axis=2)		# prepare new generation


def randomizePop(population):

	for i in range(0, popsize):		# randomize starting values of genes

		for j in range(0, len(genes)):

			population[0, i, j] = rand.sample(xrange(max(max(genes[j]))), 1)[0]		# double max to only grab one value
			if population[0, i, j] == 0:		# 0 doesnt work with findOpening
				population[0, i, j] = 1
	print 'randomized population into \n', population
	return population



def findFittest(population):		# takes the two fittest individuals

	for n in range(0,2):

		tempfit = population[0, :, len(genes)]

		for i in range(0, popsize):

			if population[0, i, len(genes)] == max(tempfit):

				foo = population[0, i, :]

				newpop[0, n, :] = foo

				population[0, i, len(genes)] = 0

				print 'fittest individual is', i

				break
	print 'after findfittest', newpop
	return newpop



def makeOffspring(newpop):
	print 'making offspring of', newpop, '...'
	for i in range(2, popsize):		# dont overwrite parents

		for j in range(0, len(genes)):		# dont overwrite fitness

			if rand.randint(1,2) == 1:

				newpop[0, i, j] = newpop[0, 0, j]		# F gene 50% chance from either P

			else:

				newpop[0, i, j] = newpop[0, 1, j]

	return newpop



def mutateGenes(newpop, amount, chance, decimals=False):

	if decimals == False:	

		amount = range(0, amount)		# so it only requires maxima in the call

	else:

		amount = np.linspace(0, amount, amount*10)		# single decimal space

	for i in range(2, popsize):		# dont mutate parents

		for j in range(0, len(genes)):		# dont overwrite fitness

			if rand.random() <= chance:		# times two to not inflate mutation chance

				newpop[0, i, j] = newpop[0, i, j] + rand.choice(amount)

				print 'mutated gene', i,j, 'into', newpop[0, i, j]

				if rand.random() <= 0.5:		# 50% to decrease in value

					newpop[0, i, j] = newpop[0, i, j] - rand.choice(amount)

					print 'mutated gene', i,j, 'into', newpop[0, i, j]

	return newpop

def findOpening(image, gene0, gene1, gene2, gene3, gene4):		# adapted for genetic script not to be recursive until it finds at least one circle!
## gene0: max size; gene1: min size; gene2: gradient edge detection; gene3: threshold; gene4: accumulator resolution ratio NEEDS DECIMALS!
    result = []
    MAX_SIZE = int(round(gene0))  # loads genes into variables; NEEDS INTEGER!
    MIN_SIZE = int(round(gene1))		# NEEDS INTEGER!
    print 'finding openings with gene values', gene0, gene1, gene2, gene3, gene4
    image = cv2.imread(image)
    output = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", gray)
    #cv2.waitKey(0)
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    #cv2.imshow("thresh", thresh)
    #cv2.waitKey(0)
    startp1 = gene2
    startp2 = gene3
    circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,gene4/10,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
## change param 1 and 2 for more-less circles
    if circles is not None:
# convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        print len(circles), 'detected'
# loop over the (x, y) coordinates and radius of the circles
        for i in range(0,len(circles)):
    # draw the circle in the output image, then draw a rectangle
    # corresponding to the center of the circle
            cv2.circle(output, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
            cv2.rectangle(output, (circles[i,0] - 5, circles[i,1] - 5), (circles[i,0] + 5, circles[i,1] + 5), (0, 128, 255), -1)
    else:
        print 'no circles detected with gene values', gene0, gene1, gene2, gene3, gene4
        circles = 'error'
# show the output image
#    cv2.imshow("output", output)
#    cv2.waitKey(0)
    print circles
    return circles

def determineFitness(population):
	tempfitness = 1100		# maximum with one circle is 1000 fitness
	if population == 'error':
		tempfitness = 0
		return tempfitness
	else:
		tempfitness = tempfitness - (len(population)*8)		# most weight placed on number of circles
		print 'fitness after number of circles', tempfitness
	tempfitness = tempfitness - abs(np.mean(population[:,0]) - 1188)
	print 'fitness after x', tempfitness
	tempfitness = tempfitness - abs(np.mean(population[:,1]) - 190)
	print 'fitness after y', tempfitness
	tempfitness = tempfitness - abs(np.mean(population[:,2]) - 130)
	print 'fitness after r', tempfitness
	if tempfitness < 0:
		tempfitness = 1		# to set apart from NO circle genes
	print 'fitness is', tempfitness
	return tempfitness

def writeFitness(newpop):
	for i in range(0, popsize):
		popfind = findOpening('curImage.png', gene0= newpop[0, i, 0], gene1= newpop[0, i, 1], gene2= newpop[0, i, 2], gene3= newpop[0, i, 3], gene4= newpop[0, i, 4])
		fitness = determineFitness(popfind)
		newpop[0, i, geneamount] = fitness
		print 'fitness written for individual', i
		print newpop
	return newpop


def runGeneration(population, iterations):
	fitmax = range(0, iterations)
	population = randomizePop(population)
	for i in range(0, iterations):
		print 'starting generation', i

		population = writeFitness(population)

		newpop = findFittest(population)

		newpop = makeOffspring(newpop)

		population = mutateGenes(newpop, 20, 0.05, decimals=False)
		print 'generation', i, 'complete:', population
		temp = max(population[0,:,len(genes)])
		fitmax[i] = temp 		# Has to be convoluted or it overwrites it
	print 'all generations complete complete:\n', population
	print fitmax[max(fitmax)]
	return fitmax

# Plots fitness by generation
iterations = 2
y = runGeneration(population, iterations)
x = np.arange(iterations)
plt.plot(x, y)
plt.show()

