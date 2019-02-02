#title           :timescaling.py
#description     :This will perform single step timescaling.
#author          :Mithun Babu Nallana
#date            :20/12/2018
#version         :0.1
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np

from computeconstraints import computeconstraints
from computescale import computescale
from scalegrid import scalegrid

def timescaling(stateRobo, radiusRobo, limitsRobo, stateObst, radiusObst, deltaT, deltaTSmall):
	
	# initial guess scale
	guessScale = 1.5

	# iteration for convexification of constraint
	iters = 3

	# weight given to constraint violation slack terms
	weightSlack = 1000

	# optimizing for scale
	#
	# a, b, c = computeconstraints(stateObst, stateRobo, radiusObst, radiusRobo, deltaT)
	# scale = computescale(a, b, c, iters,stateRobo, limitsRobo, weightSlack, guessScale, deltaT)
	scale=1
	# print('scale is ',scale)
	scaleGrid = scalegrid(scale, deltaT, deltaTSmall)
	#print(scaleGrid[1:]-scaleGrid[:-1]) 
	for i in range(len(scaleGrid)):
		scaleGrid[i]=1

	nScale = len(scaleGrid)
	velRoboNew = np.zeros(nScale-1)
	omegaRoboNew = np.zeros(nScale-1)
	for m in range(1,nScale):
		velRoboNew[m-1] = stateRobo[3]*scaleGrid[m]
		omegaRoboNew[m-1] = stateRobo[4]*scaleGrid[m]

	return velRoboNew, omegaRoboNew 

