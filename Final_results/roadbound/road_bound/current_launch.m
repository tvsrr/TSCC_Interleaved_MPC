% Simple Steering control example to show how to use M-File script wrapper
% to work with a VS solver DLL using the VS API. This version uses
% vs_statement to setup the VS Solver with import and export variables to
% match the arrays defined in MATLAB.

% To run, generate files from from the CarSim browser (use the button
% "Generate Files for this Run"), and then run this file from MATLAB.
% This is set up to look for a file named 'steer_control_vs_cmd.sim'.

% Check if last loaded library is still in memory. If so, unload it.
clear;
clc;

load data_raise.mat

%% coded bro
path_data = csvread('PathXY_left.csv');
path_data = path_data(2:end,:);
path_size = size(path_data,1);

temp = 5:5:5*path_size;
path_data = [path_data,temp'];

needed_points = 100;
station_req = linspace(0,5*path_size,needed_points+1);
p_x = interp1(path_data(:,3),path_data(:,1),station_req,'spline');
p_y = interp1(path_data(:,3),path_data(:,2),station_req,'spline');
p_x = p_x(:,2:end)';
p_y = p_y(:,2:end)';
p_s = station_req(:,2:end)';


steps_to_run = 3;

steps = 50; % number of total steps
deltaT = 0.1; % Fixing deltaT

avg_speed = 10;
s_change = p_s(2,:) - p_s(1,:);
each_step_dist = avg_speed*deltaT;
num_add = s_change/each_step_dist;
p_t = (0+num_add:num_add:needed_points*num_add)';

% Initial conditions for robot
initialX = 0;
initialY = 0;
initialAng = atan2((p_y(2)-p_y(1)),(p_x(2)-p_x(1)));
initialOmega = 0;
initialVel = 0;

time_total = steps*deltaT;
total_dist = avg_speed*time_total;

% main radius
mainRadius = 1;


%% Config_1
obstX = [];
obstY = [];
radiusObst = []+mainRadius ;
thetaObst = [];
numberObst = size(obstX,2);

% Initial Guess for alpha along path
guess = zeros(steps-1,1)/1000;

% Initital expansion point for velocity
constVelocity = 5*ones(steps-1,1);

% min turn radius
radMin = 3;



%%
if libisloaded('vs_solver')
    unloadlibrary('vs_solver');
end

% Scan simfile for DLL pathname. Change the name if it's not what you use.
simfile = 'current_launch.sim';
SolverPath = vs_dll_path(simfile);

% Load the solver DLL
[notfound, warnings] = ...
    loadlibrary(SolverPath, 'vs_api_def_m.h', 'alias', 'vs_solver');

% libfunctionsview('vs_solver'); % uncomment to see all VS API functions

% Start and read normal inputs
t = calllib('vs_solver', 'vs_setdef_and_read', simfile, 0, 0);

% activate three export variables from VS Solver
vs_statement('EXPORT', 'XCG_TM');
vs_statement('EXPORT', 'YCG_TM');
vs_statement('EXPORT', 'VXTARGET');
vs_statement('EXPORT', 'VX');
vs_statement('EXPORT', 'STATION');
vs_statement('EXPORT', 'YAW');

% activate three import variables for VS Solver
vs_statement('IMPORT', 'IMP_STEER_SW REPLACE 0');
vs_statement('IMPORT', 'IMP_SPEED REPLACE 0');

calllib('vs_solver', 'vs_initialize', t, 0, 0);
disp(calllib('vs_solver', 'vs_get_output_message'));


% Define import/export arrays (both with length 3) and pointers to them
imports = zeros(1, 2);
exports = zeros(1, 6);
p_imp = libpointer('doublePtr', imports);
p_exp = libpointer('doublePtr', exports);

% get time step and export variables from the initialization
t_dt = calllib('vs_solver', 'vs_get_tstep');
calllib('vs_solver', 'vs_copy_export_vars', p_exp);
stop = calllib('vs_solver', 'vs_error_occurred');
disp('The simulation is running...');

% This is the main simulation loop. Continue as long as stop is 0.
timer_all = (-steps_to_run* deltaT);
Epsilon = 0.00001;
temp_vars = [];
steer_ang = 0;

while ~stop
    %%
    % Update the array of exports using the pointer p_exp
    exports = get(p_exp, 'Value');
    initialX = exports(1);
    initialY = exports(2);
    initialVel = exports(4)*(5/18);
    initialOmega = ((exports(6)*(pi/180))-initialAng)/t_dt;
    initialAng = exports(6)*(pi/180);
    yaw_now = exports(6)*(pi/180);
    
    
    if(t == 0)
        store_x = [];
        store_y = [];
        store_v = [];
        store_t = [];
        initialOmega = 0;
        load_head = initialAng*(180/pi);
    end
    if(abs((t) - ((timer_all + steps_to_run*deltaT))) <= Epsilon )
        timer_all = t;
        disp(timer_all);
        axis equal;
        
        finalX = [];
        finalY = [];
        finalT = [];
        finalW = [];
        finalAng = [];
        finalOmega = [];
        finalVel = [];
        
        for i = 1:needed_points-1
            if( 0 < p_t(i) && p_t(i) < steps)
                finalX = [finalX; p_x(i)];
                finalY = [finalY; p_y(i)];
                finalT = [finalT; p_t(i)];
                finalAng = [finalAng; atan2((p_y(i+1)-p_y(i)),(p_x(i+1)-p_x(i)))];
                if(i == needed_points-1)
                    finalOmega = [0];
                    finalVel = [0];
                    fianlW = [finalW;20];
                else
                    finalW = [finalW;1];
                end
            end
        end
        jos = 1;
        for p = size(finalX,1):-1:1
            finalW(p) = jos;
            jos = jos*4;
        end
        [ v_c,v_a,alpha ] = mpc_call( steps,deltaT,initialX,initialY,initialAng,initialOmega,initialVel,finalX,finalY,finalT,finalW,finalAng,finalOmega,finalVel,mainRadius,obstX,obstY,radiusObst,thetaObst,numberObst,guess,constVelocity,radMin,op );
        temp_vel_array = [initialVel;v_c(1:steps_to_run)]*(18/5);
        half_mat = zeros(size(alpha));
        for h = 1:steps-1
            half_mat(h,:) = (steps-h-0.5)*deltaT*deltaT;
        end
        total_ang = zeros(steps-1,1);
        for m = 1:steps-1
            total_ang(m,:) = initialAng+(m*initialOmega*deltaT)+((alpha(1:m,:))'*half_mat(steps-m:end,:)); % angle at every step
        end
        temp_theta_array = [initialAng;total_ang(1:steps_to_run)]*(180/pi);
        temp_time_array = timer_all:deltaT:timer_all + (steps_to_run*deltaT);
        guess = zeros(steps-1,1)/1000;
        constVelocity = 5*ones(steps-1,1);
        p_t = p_t - steps_to_run;
        store_x = [store_x;initialX];
        store_y = [store_y;initialY];
        store_v = [store_v;initialVel];
        store_t = [store_t;initialAng*(180/pi)];
        temp_vars = [temp_vars;exports(3)];
        figure(10);
        cla;
        hold on;
        axis equal;
        plot(store_x,store_y,'rO','LineWidth',2);
        pause(0.1);
    end
    
    t = t + t_dt; % increment time
    idx = 0;
    for h = 1: size(temp_time_array,2)
        if(temp_time_array(1,h)-t >= -Epsilon )
            idx = h;
            break;
        end
    end
    l_f = 1.4;
    l_r = 2.9250;
    beta = atan(l_r*tan(steer_ang)/(l_f+l_r));
    steer_ang = atan((((temp_theta_array(idx,1))*(pi/180) - yaw_now)/(temp_time_array(1,idx)-t))*((l_f+l_r)/((initialVel+0.0001)*cos(beta))));

    % copy values into import array and set pointer for the VS solver
    pppn  = 0;
    imports(1)= steer_ang*(180/pi);
    imports(2)= temp_vel_array(idx,1);
    set(p_imp, 'Value', imports); %set pointer for array of imports
    % Call VS API integration function and exchange import and export arrays
    stop = calllib('vs_solver', 'vs_integrate_io', t, p_imp, p_exp);
    momo = 0;
end

% Terminate solver
calllib('vs_solver', 'vs_terminate_run', t);
disp('The simulation has finished.');

% Unload solver DLL
unloadlibrary('vs_solver');
