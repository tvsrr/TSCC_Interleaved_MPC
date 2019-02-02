#!/usr/bin/python3
import snakeoil3_gym
from math import cos,sin
def mydrive(c):
	S,R= c.S.d,c.R.d
	target_speed=0
	

	#R['steer']=3.14
	R['steer']= S['angle']*10 / snakeoil3_gym.PI
	R['steer']-= S['trackPos']*.10
	
	for item in S['opponents']:
	 	if (item != 200):
	 		xobs = S['xcod']+item*cos(S['opponents'].index(item)*10);
	 		yobs = S['ycod']+item*sin(S['opponents'].index(item)*10);
	 		print(xobs,yobs,item)
	 		print('-------------')	
	 		
	
	if S['speedX'] < target_speed:
		R['accel']+= 0
	elif S['speedX'] == target_speed:
		R['accel'] = 0
	else:
		R['accel']-= 0
	if S['speedX']<5:
		# R['accel']+= 1/(S['speedX']+.1)
		R['accel']+= 0
		
        
	# if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3])-(S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 5):
	# 	R['accel']-= .2

	
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




if __name__ == "__main__":
    C= snakeoil3_gym.Client(p=3101)
    for step in range(C.maxSteps,0,-1):
        C.get_servers_input()
        mydrive(C)
        C.respond_to_server()        
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


    
