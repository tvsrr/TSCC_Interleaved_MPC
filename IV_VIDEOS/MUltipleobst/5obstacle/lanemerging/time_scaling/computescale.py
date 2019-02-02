#title           :computescale.py
#author          :Mithun Babu Nallana
#date            :12/12/2018
#version         :0.1
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np
import time
from optimizescale import optimizescale


# add constraints according to the a,b,c coefficients of quadratic function

def computescale(a, b, c, iters, stateRobo, limitsRobo, weightSlack, guessScale, deltaT):
	
	nObst = len(a)
	for i in range(iters):

		listLeft = []
		listRight = []
		

		for j in range(nObst):
			
			# concave case that leads to linearization
			if (a[j]<=0 and c[j]<=0 and b[j]>=0):
				tempLeft = (float)(a[j] + b[j]/(2*guessScale))
				tempRight = - (float)(c[j] + (b[j]/2.0)*guessScale)
				listLeft.append(tempLeft)
				listRight.append(tempRight)
				#print('first',listRight)

			# convex case with a as coefficient of s^2
			elif (a[j]>=0):
				#print('value is',(b[j]**2 - 4*a[j]*c[j])/(2*a[j]))
				root1 = (float)(-b[j] + np.sqrt(b[j]**2 - 4*a[j]*c[j]))/(2*a[j])
				root2 = (float)(-b[j] - np.sqrt(b[j]**2 - 4*a[j]*c[j]))/(2*a[j])
				scaleMax = max(root1,root2)
				scaleMin = min(root1,root2)
				
				if (scaleMin>=0):
					listLeft.append(-1.0)
					listRight.append(-1*scaleMin**2)
					listLeft.append(1.0)
					listRight.append(scaleMax**2) 
				else:
					if (scaleMax<0):
						raise ValueError('Invalid constraint')
					else:
						listLeft.append(1.0)
						listRight.append(scaleMax**2)
				#print('second',listRight)

			# convex case with c as coefficient of (1/s)^2
			elif (a[j]<=0 and c[j]>=0):
				root1 = (float)(-b[j] + np.sqrt(b[j]**2 - 4*a[j]*c[j]))/(2*a[j])
				root2 = (float)(-b[j] - np.sqrt(b[j]**2 - 4*a[j]*c[j]))/(2*a[j])
				scaleMax = max(root1,root2)
				scaleMin = min(root1,root2)
				
				if(scaleMin>0):
					listLeft.append(-1.0)
					listRight.append(-1.0/scaleMin**2)
					listLeft.append(1.0)
					listRight.append(1.0/scaleMax**2)
				else:
					if (scaleMax<=0):
						raise ValueError('Invalid constraint')
					else:
						listLeft.append(1.0)
						listRight.append(1/scaleMax**2)
				#print('third',listRight)

			# satisfied for any scale greater than zero, so no constriants
			else:
				pass
				
		guessScale = optimizescale(listLeft, listRight, stateRobo, limitsRobo, weightSlack, deltaT)
		
	return guessScale
				
