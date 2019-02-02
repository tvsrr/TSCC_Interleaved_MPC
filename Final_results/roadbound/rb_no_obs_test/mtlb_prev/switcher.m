%The integration
    [stateRobo,stateObst,vlast,wlast,vsend,wsend]=launch_iros(x0,y0,x_ob,y_ob,theta_in,na,mela,ob);
    system('python3 /../time_scaling/timescalingex.py stateRobo stateObst vl wl');
    
   
    
    
    
    