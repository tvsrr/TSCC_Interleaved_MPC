#title           :scalegrid.py
#author          :Mithun Babu Nallana
#date            :20/12/2018
#version         :0.1
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np

# divide the scaled data into a reasonable scale grid

def scalegrid(scale, deltaT, deltaTSmall):
	
	scaleDiffDiff = (scale**2-1)/(2*deltaT)
	deltaTNew = 2*deltaT/(scale+1)

	nTime = int(deltaT/deltaTSmall)
	timeGrid = np.linspace(0,deltaTNew,nTime)
	deltaTNewSmall = timeGrid[1] - timeGrid[0]
	scaleGrid = np.zeros(nTime)
	scaleGrid[0] = 1.0

	for p in range(nTime-1):
		scaleGrid[p+1] = scaleGrid[p] + scaleDiffDiff*deltaTNewSmall
	
	return scaleGrid
	
