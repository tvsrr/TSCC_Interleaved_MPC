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
	# if ke <=6 :
	# 	R['steer']= S['angle']*10 / snakeoil3_gym.PI
	# 	R['steer']-= S['trackPos']*.10 
	# 	print('ke',vel)
	# else:
	R['steer']=ster 
	#print(S['xcod'],S['ycod'])
	if S['speedX'] < target_speed:
		R['accel']+= .01
	else:
		R['accel']-= .01
	if S['speedX']<10:
		#R['accel']+= 1/(S['speedX']+.1)
		R['accel']+=0.001


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
		#print(S['xcod'],S['ycod'],0,0,S['angle'],10,0)
		x0 = S['xcod']
		y0 = S['ycod']
		x_ob = 0
		y_ob = 0
		theta_in = S['angle']
		na = 10
		mela = 0
		ob=0
		for item in S['opponents']:
			if item <=30:
				x_ob = S['xcod']+item*cos(S['opponents'].index(item)*10*pi/180);
				y_ob = S['ycod']+item*sin(S['opponents'].index(item)*10*pi/180);
				ob=1
				return x0,y0,x_ob,y_ob,theta_in,na,mela,ob
	else:
		return 

#-----------------------------------------------------------------------------------------------------------------#
#new drive function for obstacle
def newdrive(c):
	S,R= c.S.d,c.R.d
	target_speed=25
	

	#R['steer']=3.14
	R['steer']= S['angle']*10 / snakeoil3_gym.PI
	R['steer']-= S['trackPos']*.10
	
	if S['speedX'] < target_speed:
		R['accel']+= 0.01
	elif S['speedX'] == target_speed:
		R['accel'] = 0.01
	else:
		R['accel']-= 0.01
	if S['speedX']<5:
		# R['accel']+= 1/(S['speedX']+.1)
		R['accel']+= 0.001


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
	return






#------------------------------------------------------------------------------------------------------------------#

if __name__ == "__main__":
	i=0

	if i==0:
		eng = matlab.engine.start_matlab()
		eng.launch_iros(249.44,174.5,240,173.5,0,10,0,1)
		eng.quit()
		velo = genfromtxt('vec.csv', delimiter=',')
		ster = genfromtxt('woc.csv', delimiter=',')

	ct =0
	Cs= [ snakeoil3_gym.Client(p=P) for P in [3101,3102] ]
	for step in range(Cs[0].maxSteps,0,-1):
		Cs[0].get_servers_input()
		Cs[1].get_servers_input()
		if (i==9):
			x0,y0,x_ob,y_ob,theta_in,na,mela,ob=mydrive(Cs[0],int(velo[i]),int(ster[i]),i,ct)
		else:
			mydrive(Cs[0],int(velo[i]),int(ster[i]),i,ct) 

		i+=1
		if (i==10):
			keyboard.press('p')
			eng = matlab.engine.start_matlab()
			eng.launch_iros(x0,y0,x_ob,y_ob,theta_in,na,mela,ob)
			eng.quit()
			velo = genfromtxt('vec.csv', delimiter=',')
			ster = genfromtxt('woc.csv', delimiter=',')
			i = 0
			ct+=1
		keyboard.release('p')
		newdrive(Cs[1])
		Cs[0].respond_to_server()
		Cs[1].respond_to_server()
		if ct == 10:
			break
	Cs[0].shutdown()
	Cs[1].shutdown()


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


    
