import matlab.engine
import numpy as np 
from timescalingex import timescalingex 
#from math import cos,sin,pi
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Agg")
import time
#stateRobo=[x0,y0,th,vn,wn]
import matplotlib.animation as manimation
#stateRobo=[x0,y0,th,vn,wn]
FFMpegWriter = manimation.writers['ffmpeg']
metadata = dict(title='Movie Test', artist='Matplotlib',
                comment='Movie support!')
writer = FFMpegWriter(fps=15, metadata=metadata)

def mpccaller(x0,y0,xob,yob,th,na,mela,ob):
	#"-nojvm -nodisplay -nosplash"
	eng = matlab.engine.start_matlab("-nojvm -nodisplay -nosplash")
	eng.addpath('/home/raghu/Desktop/Iv_2019/Bala/3vehicles/mtlb')
	vlast,wlast,vsend,wsend,v_movx,v_movy,stateRobo,stateObst,oth,r_ob,wp=eng.launch_iros(float(x0),float(y0),matlab.double(xob),matlab.double(yob),float(th),na,mela,ob, nargout=11)
	eng.quit()
	x_points_right=np.genfromtxt('/home/raghu/Desktop/Iv_2019/Bala/3vehicles/time_scaling/x_points_right.csv')
	x_points_left=np.genfromtxt('/home/raghu/Desktop/Iv_2019/Bala/3vehicles/time_scaling/x_points_left.csv')
	y_points_right=np.genfromtxt('/home/raghu/Desktop/Iv_2019/Bala/3vehicles/time_scaling/y_points_right.csv')
	y_points_left=np.genfromtxt('/home/raghu/Desktop/Iv_2019/Bala/3vehicles/time_scaling/y_points_left.csv')
	# stateRobo=np.hstack(stateRobo)
	# stateObst=np.hstack(stateObst)
	print(wp)
	vsend = np.array(vsend)
	vsend = np.transpose(vsend)
	wsend = np.array(wsend)
	wsend = np.transpose(wsend)
	stateRobo=stateRobo[0]
	c=[];v=[];t=[]
	for i in stateObst:
		for j in i:
			c.append(j)
		v.append(c)
		c=[]
	stateObst=v
	for r in r_ob:
		t.append(np.float32(r))
	r_ob=list(t[0])
	print("velocity_mpc")
	print(vsend)
	print("Omega_mpc")
	print(wsend)

	return vlast,wlast,vsend,wsend,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_ob,wp


def tscc(stateRobo,stateObst,vl,wl,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob):
	#storing output arguments
	#calling timescaling 


	# np.asarray(stateRobo)
	# print('after np',type(stateRobo))
	# print('after list',type(stateRobo))
	velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob,cum_vel,cum_om,yaw = timescalingex(list(stateRobo),list(stateObst),vl,wl,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob)

	return velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob,cum_vel,cum_om,yaw

   
def plotter(prex,prey,prox,proy,cum_vel,cum_om,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,rob,thl,yaw,wp):
    fig2=plt.figure()
    yaw=yaw*180/3.14
    t=[];l=[]
    # for w in wp:
    # 	t.append(np.float32(w[0]))
    # 	l.append(np.float32(w[1]))

    # wp = np.array([t,l])
    #print(wp)
    print("bala")
    print(cum_om)
    with writer.saving(fig2, "writer_test.mp4", 100):
	    for i in range(len(prex)):
	    	h = 3
	    	w = 2
	    	plt.plot(x_points_right,y_points_right,linestyle='solid',color='black')
	    	plt.fill_between(x_points_right,-10,y_points_right,facecolor=(0.5,0.5,0.5))
	    	plt.fill_between(x_points_left,1000,y_points_left,facecolor=(0.5,0.5,0.5))
	    	
	    	plt.plot((x_points_left+x_points_right)/2 ,(y_points_left+y_points_right)/2 ,linestyle='--',color='black')
	    	plt.plot(x_points_left,y_points_left,linestyle='solid',color='black')
	    	
	    	plt.plot(prex,prey,label='robot')
	    	#plt.plot(wp[:,1],wp[:,2])
	    	for oi in range(len(stateObst)):
	    		plt.plot(prox[oi],proy[oi],label='obstacle')
	    		plt.plot(prox[oi][i],proy[oi][i],label='obstacle')
	    	circle1 = plt.Rectangle((prex[i],prey[i]),h,w,angle=yaw[i],color='r')
	    	ax = plt.gca()
	    	ax.add_artist(circle1)
	    	
	    	for mi in range(len(stateObst)):
	    		rect2 = plt.Rectangle((prox[mi][i]-h/2,proy[mi][i]-w/2),h,w,angle=0,color='black')
	    		ax = plt.gca()
	    		ax.add_artist(rect2)
	    		ax.axis('equal')
	    		ax.axis('equal')
	    	
	    	# plt.plot(prex[i],prey[i],marker='o',markersize=10)
	    	#plt.plot(prox[i],proy[i],marker='s',markersize=10)
	    	plt.draw()
	    	axes = plt.gca()
	    	axes.set_xlim(-5, 50)
	    	axes.set_ylim(-5, 40)
	    	time.sleep(0.0005)
	    	plt.pause(1e-17)
	    	writer.grab_frame()
	    	plt.cla()
	    # plt.close(fig)
	    plt.close(fig2)
    fig3 = plt.figure()
    plt.plot(cum_vel)
    plt.xlabel('DeltaT')
    plt.ylabel('Scaled Linear velocities')
    plt.savefig('Velocityplot.png')
    plt.show()
    plt.close(fig3)
    fig4 = plt.figure()
    plt.plot(cum_om)
    plt.xlabel('DeltaT')
    plt.ylabel('Scaled Angular velocities')
    plt.savefig('omegaplot.png')
    plt.show()
    plt.close(fig4)
    return


## main code begins here ##
#mpccaller(x0,y0,xob,yob,th,na,mela,ob)
#call mpc to get the velocities and omega values

vlast,wlast,vgot,wgot,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_ob,wp=mpccaller(0,0,[350,250],[217,209],0.785,10,1,1)


#MPC only can plot mpc velocities properly as we are sending velocites once in 1 second
#xp,yp,x_ob,y_ob,tht = plotter(vgot,wgot,0.2,0,0,0,15,0,v_movx,v_movy,n_ob)
# stateRobo=[0,0,0,0.01,0]
# #The velocities and omegas obtained from MPC are passed to tscc

prex=np.array([])
prey=np.array([])
prox=[]
proy=[] 
cumyaw =np.array([])

simulationtime=10
while(simulationtime):
	velocity,omega,deltT,stateRobo,stateObst,cumxp,cumyp,cumxob,cumyob,cum_vel,cum_om,yaw=tscc(stateRobo,stateObst,vgot,wgot,x_points_right,y_points_right,x_points_left,y_points_left,v_movx,v_movy,oth,r_ob)
	
	#accumuliation and plotting
	prex=np.concatenate((prex,cumxp),axis=0)
	prey=np.concatenate((prey,cumyp),axis=0)
	cumyaw=np.concatenate((cumyaw,yaw),axis=0)
	if len(prox)==0:
		for li in range(len(stateObst)):
			prox.append(cumxob[li])
			proy.append(cumyob[li])
	else:
		for li in range(len(stateObst)):
			prox[li]=prox[li]+cumxob[li]
			proy[li]=proy[li]+cumyob[li]
	

	xob=[];yob=[]
	xl = stateRobo[0];yl = stateRobo[1]
	thl=stateRobo[2]
	for i in range(len(stateObst)):
		xob.append(stateObst[i][0])
		yob.append(stateObst[i][1])

	plotter(prex,prey,prox,proy,cum_vel,cum_om,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,r_ob,thl,cumyaw, wp)

	vlast,wlast,vgot,wgot,v_movx,v_movy,stateRobo,stateObst,x_points_right,y_points_right,x_points_left,y_points_left,oth,r_o,wp=mpccaller(xl,yl,xob,yob,thl,10,1,1)
	simulationtime-=1

print('The end')