import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.markers
from timescaling import timescaling
import time

def updater(vel,ws,ct,deltaTSmall,staterobo,stateObst,v_movx,v_movy,oth,rob):
	xp = np.zeros(len(vel))
	yp = np.zeros(len(vel))
	x0 = staterobo[0];y0=staterobo[1];thet=staterobo[2]
	
	
	xp[0]=x0+vel[0]*deltaTSmall*np.cos(thet+ws[0]*deltaTSmall)
	yp[0]=y0+vel[0]*deltaTSmall*np.sin(thet+ws[0]*deltaTSmall)
	thet= thet+ws[0]*deltaTSmall
	
	for ct in range(1,len(vel)):
		xp[ct]=xp[ct-1]+vel[ct]*deltaTSmall*np.cos(thet+ws[ct]*deltaTSmall)
		yp[ct]=yp[ct-1]+vel[ct]*deltaTSmall*np.sin(thet+ws[ct]*deltaTSmall)
		thet= thet+ws[ct]*deltaTSmall
		
	stateRobo=[xp[-1],yp[-1],thet,vel[-1],ws[-1]]

	#Obstacle state finding
	v_movx=list(v_movx[0])
	v_movy=list(v_movy[0])



	oth = list(np.transpose(oth))
	xobs=[];yobs=[]
	for k in range(len(stateObst)):
		stateobst= stateObst[k]
		x_ob=stateobst[0];y_ob=stateobst[1];
		vx = v_movx[k]*np.ones(len(vel))
		vy = v_movy[k]*np.ones(len(vel))
		ut = np.triu(np.ones(len(vel)),k=0)
		xob = x_ob*np.ones(len(vel))+vx@ut*np.cos(oth[k])
		yob = y_ob*np.ones(len(vel))+vy@ut*np.sin(oth[k])

		if v_movx[k]!=0:
			stateObst[k]= [xob[-1],yob[-1],0,v_movx[k]]
		else:
			stateObst[k]= [xob[-1],yob[-1],1.57,v_movy[k]]

		xobs.append(list(xob))
		yobs.append(list(yob))
	return stateRobo,stateObst,xp,yp,xobs,yobs


def plotter(cumxp,cumyp,cumxob,cumyob,cum_vel,cum_om,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,rob):
	
	
	#plt.legend()
	# plt.show()
    # fig = plt.figure()
    # plt.plot(cum_vel)

    # fig3 = plt.figure()
    # plt.plot(cum_om)


    fig2=plt.figure()
    for i in range(len(cumxp)):
    	
    	plt.plot(x_points_right,y_points_right,linestyle='solid',color='black')
    	plt.plot((x_points_left+x_points_right)/2 ,(y_points_left+y_points_right)/2 ,linestyle='--',color='black')
    	plt.plot(x_points_left,y_points_left,linestyle='solid',color='black')
    	plt.plot(cumxp,cumyp,label='robot')
    	for oi in range(len(stateObst)):
    		plt.plot(cumxob[oi][i],cumyob[oi][i],label='obstacle')
    	circle1 = plt.Circle((cumxp[i],cumyp[i]),1,color='blue')
    	ax = plt.gca()
    	ax.add_artist(circle1)
    	
    	for mi in range(len(stateObst)):
    		rect2 = plt.Circle((cumxob[mi][i],cumyob[mi][i]),rob[mi],color='r')
    		ax = plt.gca()
    		ax.add_artist(rect2)
    		ax.axis('equal')
    		ax.axis('equal')
    	
    	# plt.plot(cumxp[i],cumyp[i],marker='o',markersize=10)
    	#plt.plot(cumxob[i],cumyob[i],marker='s',markersize=10)
    	plt.draw()
    	axes = plt.gca()
    	axes.set_xlim(100, 180)
    	axes.set_ylim(205, 215)
    	time.sleep(0.0005)
    	plt.pause(1e-17)
    	plt.cla()
    # plt.close(fig)
    plt.close(fig2)
    # plt.close(fig3)
    return 

def timescalingex(stateRobo,stateObst,vl,wl,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob):
	

	v_movx=list(v_movx)
	v_movy=list(v_movy)
	oth=list(oth)
	ind = 999
	#print('scaling to be run on #',len(vl),'velocities')
	# print('inside',stateRobo)
	# print('stil a list?',isinstance(stateRobo, list))
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
	#stateRobo = [1, 0, 0, 3.0, 0]
	radiusRobo = 0.9

	vl = list(vl)
	wl = list(wl)
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
	radiusObst = [0.9,0.9,0.2]
	#stateObst.append([4,-6,np.pi/2,6.0]) # obstacle 1
	# for i in range(len(stateObst)):
	# 	radiusObst.append(0.9)
	# stateObst.append([35,5,0,1.2]) # obstacle 2
	# radiusObst.append(1.0)

	# limitsRobo contains limits on robot's physics
	# It is dictionary having following sub entires
	# limitsRobo['velocity'] has [velocity Minimum, velocity Maximum, velocity Desired]
	# limitsRobo['omega'] has omegaMaximum
	# limitsRobo['acceleraiton'] has [accelration-X Maximum, acceleration-X Minimum]
	# note: acceleration-Y Maximum and Minimum are made equal to accelration-X Maximum, Minimum. 
	
 #updater

	limitsRobo = {}
	limitsRobo['velocity'] = [0, 12 ,vl[0][0]]
	limitsRobo['omega'] =  wl[0][0]
	limitsRobo['acceleration'] = [-2.2,2.2]

	# deltaT is time by which we expect to be out of collision
	# deltaTSmall can be your controller frequency
	# iters is used to solve the optimization problem of scale compute if the 
	# collision cone constraint is concave
	# weightSlack is the weight provided to slack term in optimization
	# refer "Model predictive control in Autonomous Driving Based on TimeScaled Collision Cone"
	##lets plot or simulate all velocities at once
	cum_vel = np.array([])
	cum_om = np.array([])
	deltaT = 0.4
	deltaTSmall = 0.2
	ct=0
	# print(stateRobo)
	# stateRobo,stateobst = updater(vl,wl,ct,deltaT,stateRobo,stateobst)
	cumxp=np.array([]);cumyp=np.array([])
	cumxob=[];cumyob=[]

	scar = deltaT/deltaTSmall
	fat = int(len(vl)/scar) 

	#------------------------------
	# ped1 = [117,218,1.57,-1]
	# stateObst.append(ped1)
	# v_movx.append(0)
	# v_movy.append(-1)
	# oth.append(1.57)
	# print('v_movx  is ',v_movx)
	# print('v_movy is', v_movy)
	# print('oth is', oth)
	#------------------------------

	while(fat):  

		'''
		stateObst, stateRobo needs to be updated every loop
		'''

		[velocity,omega] = timescaling(stateRobo, radiusRobo, limitsRobo,stateObst, radiusObst, deltaT, deltaTSmall)
		print('----------------------------------------------------------')
		# send velcoity, omega to simulation engine
		# simulate()
		if (ct<len(vl))&((ct+int(scar))<=len(vl)):
			ct += int(scar)-1
		# get newstate



		#since it is one step staterobo[0] should suffice
		stateRobo,stateObst,xp,yp,xobs,yobs = updater(velocity,omega,ct,deltaTSmall,stateRobo,stateObst,v_movx,v_movy,oth,radiusObst )
		
		limitsRobo['velocity']=[0, 12,  vl[ct][0]]
		limitsRobo['omega'] = wl[ct][0]
		
		
		# if(time > simulationtime):
		#   endSimulation = True
		vdiff = velocity[1:]-velocity[:-1]
		#print(vdiff.shape)

		cumxp=np.concatenate((cumxp,xp),axis=0)
		cumyp=np.concatenate((cumyp,yp),axis=0)
		
		if len(cumxob)==0:
			for li in range(len(stateObst)):
				cumxob.append(xobs[li])
				cumyob.append(yobs[li])
		else:
			for li in range(len(stateObst)):
				cumxob[li]=cumxob[li]+xobs[li]
				cumyob[li]=cumyob[li]+yobs[li]
		

		for item in vdiff:
			if item>=5:
				ind = np.where(vdiff==item)
			elif item<=-5:
				ind = np.where(vdiff==item)
		if isinstance(ind,tuple):
			ct=(len(vl))
			ind = ind[0]
			cum_vel=np.concatenate((cum_vel,velocity[1:ind[0]:1]),axis=0)
			cum_om=np.concatenate((cum_om,omega[1:ind[0]:1]),axis=0)
			return 	cum_vel, cum_om, deltaT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob,cum_vel, cum_om
		
		# print('obstacle x\'s are',cumxob)
		# print('obstacle y\'s are',cumyob)

		cum_vel=np.concatenate((cum_vel,velocity),axis=0)
		cum_om=np.concatenate((cum_om,omega),axis=0)
		
		
		

		#vel,ws,delt,x0,y0,thet,x_ob,y_ob,v_movx,v_movy,n_ob
		plotter(cumxp,cumyp,cumxob,cumyob,cum_vel,cum_om,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,radiusObst )


		print(ct)
		fat = fat-1
		
	print('loop under test...')


	return cum_vel, cum_om, deltaT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob,cum_vel, cum_om