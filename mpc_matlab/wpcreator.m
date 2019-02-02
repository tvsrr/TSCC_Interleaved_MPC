%pathtowaypointmaker 
function [wp,x_points_left,y_points_left,x_points_right,y_points_right]=wpcreator(avg_speed,dt,x0,y0)
pat = load('as.csv');
p_x = pat(:,1);
ind = min(find(p_x>x0));
p_x=p_x(p_x>x0);

p_y = pat(ind:end,2);

p_s = pat(ind:end,3);
pat= [p_x,p_y,p_s-p_s(ind)];

%Road boundaries creation - 20m road 
% x_points_left=p_x;
% y_points_left=p_y+8;

% x_points_right=p_x;
% y_points_right=p_y-5;
x_points_left=load('x_points_left.csv')
x_points_right=load('x_points_right.csv')
y_points_left=load('y_points_left.csv')
y_points_right=load('y_points_right.csv')

%heuristic 5 waypoints in 50 steps 
step_dist = avg_speed*dt;
%as per heuristic for every 10mts 1 waypoint we require 
%we have data at the path
p_s = pat(:,3);
needed_pts = round(p_s(end)/10);
pts = [10:10:10*needed_pts-1]';
p_x = pat(2:end,1);
p_y = pat(2:end,2);
p_s = p_s(2:end);
np_x=interp1(p_s,p_x,pts,'spline');
np_y=interp1(p_s,p_y,pts,'spline');
wp = [np_x,np_y,pts];
end
