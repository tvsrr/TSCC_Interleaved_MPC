#title           :updatestate.py
#author          :Mithun Babu Nallana
#date            :12/12/2018
#version         :0.1
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np

# state update considering simple differential drive motion model
#
def updatestate(currentX, currentY, currentTheta, currentVel, currentOmega, deltaT):
	
	nextTheta = currentTheta + currentOmega*deltaT
	nextX = currentX + currentVel*np.cos(nextTheta)*deltaT
	nextY = currentY + currentVel*np.sin(nextTheta)*deltaT

	return nextX, nextY, nextTheta