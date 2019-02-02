import matlab.engine
import numpy as np 
from timescalingex import timescalingex 
#from math import cos,sin,pi
import matplotlib.pyplot as plt

def coordinate_calc(stateRobo,velocity,omega,deltT):
	xf = stateRobo[0]
	yf = stateRobo[1]
	th = stateRobo[3]	
	delt=0.1
	xnai = np.zeros(len(velocity))
	ynai = np.zeros(len(velocity))
	xnai[0]=xf+velocity[0]*delt*np.cos(th+omega[0]*delt)   
	ynai[0]=yf+velocity[0]*delt*np.sin(th+omega[0]*delt)
	tht = th
	for i in range(1,len(velocity)):
		tht=tht+omega[i-1]*delt
		xnai[i]=xnai[i-1]+velocity[i]*delt*np.cos(tht+omega[i]*delt)
		ynai[i]=ynai[i-1]+velocity[i]*delt*np.sin(tht+omega[i]*delt)
	xob = stateObst[0]+stateObst[3]*deltT;
	yob = stateObst[1]+stateObst[3]*deltT;
	xl=xnai[-1]
	yl=ynai[-1]
	return xnai,ynai,xob,yob,xl,yl

def mpccaller(x0,y0,xob,yob,th,na,mela,ob):
	eng = matlab.engine.start_matlab()
	eng.addpath(r'/home/raghu/Documents/switching_main/mtlb')
	stateRobo,stateObst,vlast,wlast,vsend,wsend,v_movx,v_movy=eng.launch_iros(float(x0),float(y0),float(xob),float(yob),th,na,mela,ob, nargout=8)
	eng.quit()
	stateRobo=np.hstack(stateRobo)
	stateObst=np.hstack(stateObst)
	vsend = np.array(vsend)
	vsend = np.transpose(vsend)
	wsend = np.array(wsend)
	wsend = np.transpose(wsend)
	return stateRobo,stateObst,vlast,wlast,vsend,wsend,v_movx,v_movy


def tscc(ends,stateRobo,stateObst,vl,wl):
	#storing output arguments
	vl = float(vlast)
	wl=float(wlast)
	#calling timescaling 
	velocity,omega,deltT = timescalingex(ends,stateRobo,stateObst,vl,wl)
	return velocity,omega,deltT

def plotter(vel,ws,delt,x0,y0,thet,x_ob,y_ob,v_movx,v_movy,n_ob):
	global xn,yn,xop,yop
	if x0==0:
		xn=[];yn=[]
		xop=[];yop=[]
		xn = np.array(xn);yn = np.array(yn)
		xop = np.array(xop);yop=np.array(yop)
	xp = np.zeros(len(vel))
	yp = np.zeros(len(vel))
	xob= np.zeros(len(vel))
	yob= np.zeros(len(vel))
	xp[0]=x0+vel[0]*delt*np.cos(thet+ws[0]*delt)
	yp[0]=y0+vel[0]*delt*np.sin(thet+ws[0]*delt)
	tht = thet
	for i in range(1,len(vel)):
		tht=tht+ws[i-1]*delt
		xp[i]=xp[i-1]+vel[i-1]*delt*np.cos(tht+ws[i-1]*delt)
		yp[i]=yp[i-1]+vel[i-1]*delt*np.sin(tht+ws[i-1]*delt)
	me = np.zeros(n_ob)
	for ko in range(n_ob):
		me[ko]=ko
	#chnage here in case of multiple obstacles
	#check number of obstacles and number of velocities in v_movx
	xob = x_ob*np.ones(len(vel))+me*v_movx*np.cos(0)*delt
	yob = y_ob*np.ones(len(vel))+me*v_movy*np.sin(0)*delt
	xn = np.concatenate((xn,xp),axis=0)
	yn = np.concatenate((yn,yp),axis=0)
	xop = np.concatenate((xop,xob),axis=0)
	yop = np.concatenate((yop,yob),axis=0)
	print('xn\'s length now is',len(xn))
	# print('I am in plotter')
	plt.plot(xn,yn,label='robot')
	plt.plot(xop,yop,label='obstacle')
	plt.legend()
	plt.show()
	return xp,yp,xob,yob,tht

#Main code 

stateRobo,stateObst,vlast,wlast,vsend,wsend,v_movx,v_movy=mpccaller(0,0,15,0,0,10,1,1)
# vsend = np.array(vsend)
# vsend = np.transpose(vsend)
# wsend = np.array(wsend)
# wsend = np.transpose(wsend)
ct=0
statrtsim=1
# print(vsend)
n_ob = len(vsend)
xp,yp,x_ob,y_ob,tht = plotter(vsend,wsend,0.1,0,0,0,15,0,v_movx,v_movy,n_ob)
while(statrtsim):
	
	ends=0
	
	velocity,omega,deltT=tscc(ends,stateRobo,stateObst,vlast,wlast)
	n_ob = len(velocity)
	xp,yp,x_ob,y_ob,tht = plotter(velocity,omega,0.1,xp[-1],yp[-1],tht,x_ob[-1],y_ob[-1],stateObst[3],stateObst[3],n_ob)

	xnai,ynai,xob,yob,xl,yl = coordinate_calc(stateRobo,velocity,omega,deltT)
	vltc=vlast
	wltc=wlast
	stateRobo,stateObst,vlast,wlast,vsend,wsend,v_movx,v_movy=mpccaller(xl,yl,xob,yob,0,10,1,1)
	if vsend!=0.0:
		n_ob = len(vsend)
		xp,yp,x_ob,y_ob,tht = plotter(vsend,wsend,0.1,xp[-1],yp[-1],tht,x_ob[-1],y_ob[-1],v_movx,v_movy,n_ob)
	elif vsend==0.0:
		vlast=vltc
		wlast=wltc
		stateRobo[3]=vltc
		stateRobo[4]=wltc
		print('xp[-1] is',xp[-1])
	ct+=1
	# print(ct)
	if ct==10:
		statrtsim=0


print('I came back! Things are good :)')



