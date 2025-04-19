%pathtowaypointmaker 
function [Waypoints,roadBoundaries]=Create_Waypoints(x0)

fprintf('Creating waypoints and loading boundaries...\n');

%output variables 
roadBoundaries = struct('leftX', [], 'leftY', [], 'rightX', [], 'rightY', []);
run('./utils/set_parameters.m');

% loading the data file from csv 
% data should be of format x,y,distance travelled.
road_data_array = load('./data/road_data.csv');
p_x = road_data_array(:,1);
ind = min(find(p_x>x0));
p_x = road_data_array(ind:end,1);
p_y = road_data_array(ind:end,2);
p_s = road_data_array(ind:end,3);


%waypoint spacing strategy 
% I want my waypoints spaced apart by the distance I expect the robot to 
% cover in one control/simulation cycle (dt) when moving at its typical speed.

road_data_array= [p_x,p_y,p_s-p_s(ind)];

roadBoundaries.leftX=load('./data/x_points_left.csv');
roadBoundaries.rightX=load('./data/x_points_right.csv');
roadBoundaries.leftY=load('./data/y_points_left.csv');
roadBoundaries.rightY=load('./data/y_points_right.csv');

%heuristic 5 waypoints in 50 steps 
step_dist = config.robot.AverageSpeed*config.simulation.dt; 

%as per heuristic for every 10mts 1 waypoint we require 
%we have data at the path

pts = [step_dist:step_dist: road_data_array(:,3)]';

interpolated_x = interp1(road_data_array(:,3), road_data_array(:,1), pts, 'spline');
interpolated_y = interp1(road_data_array(:,3), road_data_array(:,2), pts, 'spline');
Waypoints = [interpolated_x,interpolated_y,pts];
end
