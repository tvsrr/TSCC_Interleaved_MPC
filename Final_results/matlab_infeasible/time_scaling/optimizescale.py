#title           :optimizescale.py
#author          :Mithun Babu Nallana
#date            :20/12/2018
#version         :0.1
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np
from cvxopt import solvers
import cvxopt

def optimizescale(listLeft, listRight, stateRobo, limitsRobo, weightSlack, deltaT):
	
	thetaRobo = stateRobo[2]
	velRobo = stateRobo[3]
	omegaRobo = stateRobo[4]
		
	velMin = limitsRobo['velocity'][0]
	velMax = limitsRobo['velocity'][1]
	velDesired = limitsRobo['velocity'][2]
	omegaMinMax = limitsRobo['omega']
	accelXMin = limitsRobo['acceleration'][0]
	accelXMax = limitsRobo['acceleration'][1]
	accelYMin = accelXMin
	accelYMax = accelXMax	

	sMax = velMax/velRobo
	sMin = velMin/velRobo

	sDesired = velDesired/velRobo
	sMaxTheta = omegaMinMax/(abs(omegaRobo) + 0.00000001)

	xdotRoboDiff = -velRobo*np.sin(thetaRobo)*omegaRobo
	ydotRoboDiff = velRobo*np.cos(thetaRobo)*omegaRobo
	xdotRobo = velRobo*np.cos(thetaRobo)
	ydotRobo = velRobo*np.sin(thetaRobo)
	
	## building constraints in Gx <= h form
	# list to numpy array
	matLeft = np.asarray(listLeft)
	matRight = np.asarray(listRight)
	nCons = len(matLeft)
	matLeft = matLeft.reshape((nCons,1))
	matRight = matRight.reshape((nCons,1)) 
	
	# collision cone constraints with slack
	ccIneqLeft = np.hstack((matLeft, -np.identity(nCons)))
	ccIneqRight = matRight

	# positivity of slack constraint
	# positivity of scale constraint
	posIneqLeft = -np.identity(nCons+1)
	posIneqRight = np.zeros((nCons+1, 1)) #just in case of error change here 

	# maximum limit on scale by velocity limit
	svMaxIneqLeft = np.zeros((1, nCons+1))
	svMaxIneqLeft[0][0] = 1.0
	svMaxIneqRight = np.array([[sMax**2]])

	# minimum limit on scale by velocity limit
	svMinIneqLeft = np.zeros((1, nCons+1))
	svMinIneqLeft[0][0] = -1.0
	svMinIneqRight = -(sMin**2)*np.ones((1,1))
	
	# maximum limit on scale by omega limit
	soMaxIneqLeft = np.zeros((1, nCons+1))
	soMaxIneqLeft[0][0] = 1.0
	soMaxIneqRight = (sMaxTheta**2)*np.ones((1,1))

	tempCoeff = (xdotRobo/(2*deltaT))
	# maximum limit on scale by x-accleration limit
	saxMaxIneqLeft = np.hstack((tempCoeff*np.ones((1,1)), np.zeros((1, nCons))))
	saxMaxIneqRight = (accelXMax + tempCoeff - xdotRoboDiff)*np.ones((1,1))

	# minimum limit on scale by x-acceleration limit
	saxMinIneqLeft = np.hstack((-tempCoeff*np.ones((1,1)), np.zeros((1, nCons))))
	saxMinIneqRight = (-accelXMin - tempCoeff + xdotRoboDiff)*np.ones((1,1))

	tempCoeff = (ydotRobo/(2*deltaT))
	# maximum limit on scale by y-acceleration limit
	sayMaxIneqLeft = np.hstack((tempCoeff*np.ones((1,1)), np.zeros((1, nCons))))
	sayMaxIneqRight = (accelYMax + tempCoeff - ydotRoboDiff)*np.ones((1,1))
	
	# minimum limit on scale by y-acceleration limit
	sayMinIneqLeft = np.hstack((-tempCoeff*np.ones((1,1)), np.zeros((1, nCons))))
	sayMinIneqRight = (-accelYMin -tempCoeff + ydotRoboDiff)*np.ones((1,1))

	# combining all inequalities
	G = np.vstack((ccIneqLeft, posIneqLeft, svMaxIneqLeft, 
						svMinIneqLeft, soMaxIneqLeft, saxMaxIneqLeft, 
						saxMinIneqLeft, sayMaxIneqLeft, sayMinIneqLeft))

	# print('ccIneqRight',len(ccIneqRight))
	# print('posIneqRight',len(posIneqRight))
	# print('svMaxIneqRight',len(svMaxIneqRight))
	# print('svMinIneqRight',len(svMinIneqRight))
	# print('soMaxIneqRight',len(soMaxIneqRight))
	# print('saxMaxIneqRight',len(saxMaxIneqRight))
	# print('saxMinIneqRight',len(saxMinIneqRight))
	# print('sayMaxIneqRight',len(sayMaxIneqRight))
	# print('sayMinIneqRight',len(sayMinIneqRight))
	# print('and value is',posIneqRight)
	
	h = np.vstack((ccIneqRight, posIneqRight, svMaxIneqRight,
						svMinIneqRight, soMaxIneqRight, saxMaxIneqRight, 
						saxMinIneqRight, sayMaxIneqRight, sayMinIneqRight))

	## building cost function in (1/2)x'*P*x + q'*x (quadratic)
	# scale preference
	sDesiredArr = np.array([[sDesired**2]])
	scaleArr = np.zeros((1, nCons+1))
	scaleArr[0][0] = 1.0
	scaleP = np.dot(np.transpose(scaleArr), scaleArr)
	scaleq = -np.dot(np.transpose(scaleArr), sDesiredArr)
	
	# weighed slack terms
	slackArr = np.ones((nCons+1, 1))
	slackArr[0][0] = 0.0
	slackq = slackArr
	P = scaleP
	q = (scaleq + 2*weightSlack*slackq)
	# print("q=")
	# print(scaleq.shape)
	# print(scaleq.size)
	# print(slackq.shape)
	# print(slackq.size)
	# print('q\'s datatype is',q.dtype,'length is ',len(q) )

	# print('type is ',(G))
	
	solvers.options['feastol'] = 0.01
	solvers.options['show_progress'] = False
	sol = solvers.qp(cvxopt.matrix(P, tc='d'), 
			cvxopt.matrix(q, tc='d'), 
			cvxopt.matrix(G, tc='d'), 
			cvxopt.matrix(h, tc='d'), None, None)

	optimalSoln = np.array(sol['x'])
	optimalScale = np.sqrt(optimalSoln[0][0])
	
	# constraintViolation = max(optimalSoln[1:len(optimalSoln)])
	# print(constraintViolation)
	# print(optimalScale)
	
	return optimalScale