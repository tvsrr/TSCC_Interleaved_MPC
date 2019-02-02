import matlab.engine
import numpy as np 
from timescalingex import timescalingex 
#from math import cos,sin,pi
import matplotlib.pyplot as plt

#stateRobo=[x0,y0,th,vn,wn]


def mpccaller(x0,y0,xob,yob,th,na,mela,ob):
	eng = matlab.engine.start_matlab("-nojvm -nodisplay -nosplash")
	eng.addpath(r'/home/raghu/Desktop/Iv_2019/matlab_infeasible/mtlb')
	vlast,wlast,vsend,wsend,v_movx,v_movy,stateRobo,stateObst,oth,r_ob=eng.launch_iros(float(x0),float(y0),matlab.double(xob),matlab.double(yob),th,na,mela,ob, nargout=10)
	eng.quit()
	x_points_right=np.genfromtxt('/home/raghu/Desktop/Iv_2019/matlab_infeasible/time_scaling/x_points_right.csv')
	x_points_left=np.genfromtxt('/home/raghu/Desktop/Iv_2019/matlab_infeasible/time_scaling/x_points_left.csv')
	y_points_right=np.genfromtxt('/home/raghu/Desktop/Iv_2019/matlab_infeasible/time_scaling/y_points_right.csv')
	y_points_left=np.genfromtxt('/home/raghu/Desktop/Iv_2019/matlab_infeasible/time_scaling/y_points_left.csv')
	# stateRobo=np.hstack(stateRobo)
	# stateObst=np.hstack(stateObst)
	
	vsend = np.array(vsend)
	vsend = np.transpose(vsend)
	wsend = np.array(wsend)
	wsend = np.transpose(wsend)
	stateRobo=stateRobo[0]
	c=[];v=[]
	for i in stateObst:
		for j in i:
			c.append(j)
		v.append(c)
		c=[]
	stateObst=v
	return vlast,wlast,vsend,wsend,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_ob


def tscc(stateRobo,stateObst,vl,wl,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob):
	#storing output arguments
	#calling timescaling 


	# np.asarray(stateRobo)
	# print('after np',type(stateRobo))
	# print('after list',type(stateRobo))
	velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob = timescalingex(list(stateRobo),list(stateObst),vl,wl,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob)
	
	return velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob

def updater(vel,ws,delt,x0,y0,thet,x_ob,y_ob,v_movx,v_movy,n_ob):
	if x0==0:
		xn=[];yn=[]
		xop=[];yop=[]
		xn = np.array(xn);yn = np.array(yn)
		xop = np.array(xop);yop=np.array(yop)
	xp = np.zeros(len(vel))
	yp = np.zeros(len(vel))
	xob= np.zeros(len(vel))
	yob= np.zeros(len(vel))
	xp[0]=x0
	yp[0]=y0
	tht = thet
	for i in range(1,len(vel)):
		xp[i]=xp[i-1]+vel[i-1]*delt*np.cos(tht+ws[i-1]*delt)
		yp[i]=yp[i-1]+vel[i-1]*delt*np.sin(tht+ws[i-1]*delt)
		tht=tht+ws[i-1]*delt
	me = np.zeros(n_ob)
	for ko in range(n_ob):
		me[ko]=ko

	#chnage here in case of multiple obstacles
	#check number of obstacles and number of velocities in v_movx
	xob = x_ob*np.ones(len(vel))+me*v_movx*np.cos(0)
	yob = y_ob*np.ones(len(vel))+me*v_movy*np.sin(0)
	xn = np.concatenate((xn,xp),axis=0)
	yn = np.concatenate((yn,yp),axis=0)
	xop = np.concatenate((xop,xob),axis=0)
	yop = np.concatenate((yop,yob),axis=0)
	# print('I am in plotter')
	return xp,yp,xob,yob,tht


## main code begins here ##
#mpccaller(x0,y0,xob,yob,th,na,mela,ob)

#call mpc to get the velocities and omega values
vlast,wlast,vgot,wgot,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_ob=mpccaller(109,211,[120,100],[210,200],0,10,1,1)


#MPC only can plot mpc velocities properly as we are sending velocites once in 1 second

#xp,yp,x_ob,y_ob,tht = plotter(vgot,wgot,0.2,0,0,0,15,0,v_movx,v_movy,n_ob)

# stateRobo=[0,0,0,0.01,0]
# #The velocities and omegas obtained from MPC are passed to tscc

prex=[]
prey=[]
prox=[]
proy=[] 


simulationtime=10
while(simulationtime):
	velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob=tscc(stateRobo,stateObst,vgot,wgot,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob)

	prex=cumxp
	prey=cumyp
	prox=cumxob
	proy=cumyob 

	n_ob = len(vgot)

	# xp,yp,x_ob,y_ob,tht = updater(velocity,omega,0.1,0,0,0,15,0,v_movx,v_movy,n_ob)

	# n_ob = len(velocity)
	xob=[];yob=[]
	xl = stateRobo[0];yl = stateRobo[1]
	for i in range(len(stateObst)):
		xob.append(stateObst[i][0])
		yob.append(stateObst[i][1])
    
	fig3=plt.figure()
	plt.plot(cumxp,cumyp)
	for i in range(len(stateObst)):
		plt.plot(cumxob[i],cumyob[i])
	plt.pause(0.1)
	plt.close(fig3)



	vlast,wlast,vgot,wgot,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_ob=mpccaller(xl,yl,xob,yob,0,10,1,1)
	simulationtime-=1

print('The end')