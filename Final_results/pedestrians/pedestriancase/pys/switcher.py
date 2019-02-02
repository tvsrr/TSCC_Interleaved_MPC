import matlab.engine
import numpy as np 
from timescalingex import timescalingex 
#calling MPC 
eng = matlab.engine.start_matlab()
eng.addpath(r'/home/raghu/Documents/switching_main/mtlb')
[stateRobo,stateObst,vlast,wlast,vsend,wsend]=eng.launch_iros(0,0,15,0,0,10,1,1)
eng.quit()

#storing output arguments
stateRobo=np.array(stateRobo)
stateObst=np.array(stateObst)
vsend = np.array(vsend)
wsend = np.array(wsend)
vl = float(vlast)
wl=float(wlast)
ends=0

#calling timescaling 
velocity,omega = timescalingex(ends,stateRobo,stateObst,vl,wl)

print('I came back! Things are good :)')