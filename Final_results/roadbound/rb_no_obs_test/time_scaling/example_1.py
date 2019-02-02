#title           :example_1.py
#description     :Run reactive agent using timescaling.
#author          :Mithun Babu Nallana
#date            :20/12/2018
#version         :0.1
#usage           :python example_1.py
#notes           :additional_material.pdf
#python_version  :3.6.7

import numpy as np
from timescaling import timescaling


endSimulation = False

# stateRobo contains robot state information
# robot state has 5 entires
# [r1,r2,r3,r4,r5]
# r1 : robot's x-coordinate
# r2 : robot's y-coordinate
# r3 : robot's heading direction
# r4 : robot's velocity
# r5 : robot's omega/angular-velocity
#
# radius Robo contains robot radius information
#
stateRobo = [0, 0, 0, 0.01, 0]
radiusRobo = 0.2

# stateObst contains obstacle state information
# each obstacle state has 4 entries as
# [o1,o2,o3,o4]
# o1 : obstacle x-coordinate
# o2 : obstacle y-coordinate
# o3 : obstacle heading direction
# o4 : obstacle velocity
#
# radiusObst contains obstacle radius information
# for each obstacle we have a corresponding radius entry
#
stateObst = []
radiusObst = []
# stateObst.append([4,-6,np.pi/2,6.0]) # obstacle 1
stateObst.append([15,0,0,1])
print('working how?',len(stateObst))
radiusObst.append(0.2)
# stateObst.append([35,5,0,1.2]) # obstacle 2
# radiusObst.append(1.0)

# limitsRobo contains limits on robot's physics
# It is dictionary having following sub entires
# limitsRobo['velocity'] has [velocity Minimum, velocity Maximum, velocity Desired]
# limitsRobo['omega'] has omegaMaximum
# limitsRobo['acceleraiton'] has [accelration-X Maximum, acceleration-X Minimum]
# note: acceleration-Y Maximum and Minimum are made equal to accelration-X Maximum, Minimum. 
#
limitsRobo = {}
limitsRobo['velocity'] = [4, 15.0, 10.31003803]
limitsRobo['omega'] = 0.4
limitsRobo['acceleration'] = [-6.0,6.0]

# deltaT is time by which we expect to be out of collision
# deltaTSmall can be your controller frequency
# iters is used to solve the optimization problem of scale compute if the 
# collision cone constraint is concave
# weightSlack is the weight provided to slack term in optimization
# refer "Model predictive control in Autonomous Driving Based on TimeScaled Collision Cone"
# 
deltaT = 0.4
deltaTSmall = 0.2

lpcountn = 0

while(not endSimulation):

    '''
    stateObst, stateRobo needs to be updated every loop
    '''

    [velocity, omega] = timescaling(stateRobo, radiusRobo, limitsRobo, \
                        stateObst, radiusObst, deltaT, deltaTSmall)

    
    # send velcoity, omega to simulation engine
    # simulate()
    
    # get newstate
    # stateRobo = updated()
    # stateObst = updated()
    
    np.savetxt("vec.csv", velocity, delimiter=",")
    np.savetxt("woc.csv", omega, delimiter=",")

    #if(time > simulationtime):

    endSimulation = True
