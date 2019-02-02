#!/usr/bin/python3
import snakeoil3_gym
from numpy import genfromtxt
import matlab.engine 
from pynput.keyboard import Key
import keyboard
import pyautogui
from math import cos,sin,pi

def mydrive(c,vel,ster,fi,ke):
	S,R= c.S.d,c.R.d
	target_speed=vel
	if ke <=6 :
		R['steer']= S['angle']*10 / snakeoil3_gym.PI
		R['steer']-= S['trackPos']*.10 
		print('ke',vel)
	else:
		R['steer']=ster 
	#print(S['xcod'],S['ycod'])
	if S['speedX'] < target_speed - (R['steer']*50):
		R['accel']+= .01
	else:
		R['accel']-= .01
	if S['speedX']<10:
		#R['accel']+= 1/(S['speedX']+.1)
		R['accel']+=0.02

	if ke == 9 :


	if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 5):
		R['accel']-= .2

	R['gear']=1
	if S['speedX']>50:
		R['gear']=2
	if S['speedX']>80:
		R['gear']=3
	if S['speedX']>110:
		R['gear']=4
	if S['speedX']>140:
		R['gear']=5
	if S['speedX']>170:
		R['gear']=6
	if(fi==9):	
		print(S['xcod'],S['ycod'],0,0,S['angle'],10,0)
		x0 = S['xcod']
		y0 = S['ycod']
		x_ob = 0
		y_ob = 0
		theta_in = S['angle']
		na = 10
		mela = 0
		ob=0
		if item in S['opponents']<30:
			x_ob = S['xcod']+S['opponents']*cos(S['opponents'].index(item)*10*pi/180);
			y_ob = S['ycod']+S['opponents']*sin(S['opponents'].index(item)*10*pi/180);
			ob=1
		return x0,y0,x_ob,y_ob,theta_in,na,mela,ob
	else:
		return 






if __name__ == "__main__":
	i=0

	if i==0:
		eng = matlab.engine.start_matlab()
		eng.launch_iros(85.3066,19.0348,0,0,0,10,1)
		eng.quit()
		velo = genfromtxt('vec.csv', delimiter=',')
		ster = genfromtxt('woc.csv', delimiter=',')

	ct =0 	
	C= snakeoil3_gym.Client(p=3101)
	for step in range(C.maxSteps,0,-1):
		C.get_servers_input()
		if (i==9):
			x0,y0,x_ob,y_ob,theta_in,na,mela,ob=mydrive(C,int(velo[i]),int(ster[i]),i,ct)
		else:
			mydrive(C,int(velo[i]),int(ster[i]),i,ct) 
		i+=1
		if (i==10):
			keyboard.press('p')
			eng = matlab.engine.start_matlab()
			eng.launch_iros(y0,x0,x_ob,y_ob,theta_in,na,mela,ob)
			eng.quit()
			velo = genfromtxt('vec.csv', delimiter=',')
			ster = genfromtxt('woc.csv', delimiter=',')
			i = 0
			ct+=1
		keyboard.release('p')
		C.respond_to_server()
		if ct == 10:
			break
	C.shutdown()


# #!/usr/bin/python
# import snakeoil3_gym
# if __name__ == "__main__":
#     Cs= [ snakeoil3_gym.Client(p=P) for P in [3101,3102,3103,3104] ]
#     for step in range(Cs[0].maxSteps,0,-1):
#         for C in Cs:
#             C.get_servers_input()
#             mydrive(C)
#             C.respond_to_server()
#     else:
#         for C in Cs: C.shutdown()