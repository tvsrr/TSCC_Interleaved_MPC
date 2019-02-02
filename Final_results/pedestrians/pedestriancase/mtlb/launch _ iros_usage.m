%launch file 
%contains all the necessary function- calls
%The dependant functions are linerisation of the values,finding A,Q,C
% Formulatng the cost functons 
%This document will have the cvx- optimization
%Rectangles will be a function
%Obstacle avoidance for 3 circles will be afunction 
%Plotting will be another function
clear all;
clc;
 [x0,y0,xg,yg,w_guess,v_guess,theta_in,theta_f,n,delt,x_points_left,y_points_left,x_points_right,y_points_right] = get_input();
 ct=20;
while(ct)
 [coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,x_cod,y_cod,const_x,const_y,w_x]= linearise_hol(n,x0,y0,w_guess,v_guess ,delt,theta_in);
 num = 40;
 [A,Q,C]= mat_creator(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,num,xg,yg);


 
 %% optimization
cvx_begin quiet
variables w_cvx(1,n) vel(1,n)
new= [w_cvx,vel];
fin3=theta_in+sum(w_cvx)*delt;
fin = new*A*new'+Q*new'+C+(fin3-theta_f)^2+ 10*(w_cvx(1)-0)^2 + 10*(w_cvx(end)-0)^2+(10*(vel(1)-0)^2 + 10*(vel(end)-0)^2);
minimize(fin) 

subject to 
-0.5<=w_cvx<=0.5;
0<=vel<=5;
v_guess - 0.5 <= vel <= v_guess + 0.5;
w_guess-0.5<=w_cvx<=w_guess+0.5;
-2.2*delt <= w_cvx(2:n) - w_cvx(1:(n-1))<= 2.2*delt;
-2.2*delt <= vel(2:n) - vel(1:(n-1))<= 2.2*delt;
-0.05<= w_cvx(1)<=0.05;
-0.05<= w_cvx(n)<=0.05;
cvx_end
v_guess = vel;
w_guess = w_cvx;
cvx_optval
ct = ct-1
[x1,y1]=plot_karo(x0,y0,xg,yg,v_guess,w_guess,theta_in,delt,n);
end

