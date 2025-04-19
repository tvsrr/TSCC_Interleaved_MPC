%Input values 
function [w_guess,v_guess,delt,roadBoundaries,Waypoints,n_w,n_ob,v_obx,v_oby,r_ob,n,avg_speed,oth] = get_input(x0,y0)
n=60;

w_guess=0.1*ones(1,n); 
v_guess=5*ones(1,n);
theta_in=0;
theta_f=0;

delt= 0.2;
avg_speed=10;

[Waypoints,roadBoundaries] = wpcreator(x0);


n_w=Waypoints(:,3);


end
