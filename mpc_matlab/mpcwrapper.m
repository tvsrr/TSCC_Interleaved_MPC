% File to Run MPC Matlab
% Initial Point of the Ego Vehicle
x0 = 0;
y0 = 0;

% Obstacle position
x_ob = 25;
y_ob = 0;

% Initial Theta of the ego vehicle
theta_in = 0;

% Other flags and parameters
ctrl_horizon = 10;
stop_obstacle_iteration = 1;
enable_obstacle_avoidance = 1;

% Main Function that runs MPC
[stateRobo,stateObst,vlast,wlast,...
    vsend,wsend,v_movx,v_movy]        =   run_mpc(x0, y0, x_ob,...
                                                      y_ob, theta_in,...
                                                      ctrl_horizon, stop_obstacle_iteration,...
                                                      enable_obstacle_avoidance);
                                                                    
