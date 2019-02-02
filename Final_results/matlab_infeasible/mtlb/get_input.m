%Input values 
function [w_guess,v_guess,delt,x_points_left,y_points_left,x_points_right,y_points_right,wp,n_w,n_ob,v_obx,v_oby,r_ob,n,avg_speed,oth] = get_input(x0,y0)
n=50;
w_guess=0.1*ones(1,n); 
v_guess=5*ones(1,n);
theta_in=0;
theta_f=0;

% x_points_left=linspace(0,50,100)';
% y_points_left=5*ones(1,size(x_points_left,1))';
% 
% x_points_right=linspace(0,50,100)';
% y_points_right=-5*ones(1,size(x_points_right,1))';

delt= 0.2;
avg_speed=10;

[wp,x_points_left,y_points_left,x_points_right,y_points_right] = wpcreator(avg_speed,delt,x0);
% 	csvwrite('wp2.csv',wp);
if x0==109
    %disp('changed')
 	csvwrite('x_points_left.csv',x_points_left);
 	csvwrite('x_points_right.csv',x_points_right);
    csvwrite('y_points_right.csv',y_points_right);
 	csvwrite('y_points_left.csv',y_points_left);
end
% else
% 	wp = load('wp2.csv');

%    x_points_left=load('x_points_left.csv');
% 	x_points_right=load('x_points_right.csv');
% 	y_points_left=load('y_points_left.csv');
% 	y_points_right=load('y_points_right.csv');
% end
n_w=wp(:,3);
r_ob=[0.9,0.9,0.2];
oth = [0,0,1.57];
n_ob=1000;
v_obx=[0.2,0.2];
v_oby=[0,0];


end