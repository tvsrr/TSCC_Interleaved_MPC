% Clear workspace and command window
clear all;
clc;

% Get input parameters
[start_x, start_y, goal_x, goal_y, angular_rate_guess, velocity_guess, start_theta, end_theta, num_points, time_step, left_x_points, left_y_points, right_x_points, right_y_points] = get_input();

% Initialize optimization iteration counter
iteration_counter = 20;

% Start optimization iteration
while(iteration_counter)
    % Linearize the non-holonomic system
    [x_coeffs_final, x_coeffs_velocity, y_coeffs_final, y_coeffs_velocity, x_const, y_const, angular_rate_x]= linearise_hol(num_points, start_x, start_y, angular_rate_guess, velocity_guess, time_step, start_theta);

    % Create matrices for optimization
    [A_matrix, Q_vector, C_scalar] = mat_creator(x_coeffs_final, x_coeffs_velocity, y_coeffs_final, y_coeffs_velocity, x_const, y_const, num_points, goal_x, goal_y);
 
    %% Optimization using CVX
    cvx_begin quiet
        % Define optimization variables
        variables angular_rate_cvx(1, num_points) velocity(1, num_points)
        new= [angular_rate_cvx, velocity];
        
        % Define cost function
        final_theta = start_theta + sum(angular_rate_cvx) * time_step;
        cost_function = new * A_matrix * new' + Q_vector * new' + C_scalar + (final_theta - end_theta)^2 + 10 * (angular_rate_cvx(1) - 0)^2 + 10 * (angular_rate_cvx(end) - 0)^2 + (10 * (velocity(1) - 0)^2 + 10 * (velocity(end) - 0)^2);
        
        % Minimize cost function
        minimize(cost_function) 

        % Define constraints
        subject to 
            -0.5 <= angular_rate_cvx <= 0.5;
            0 <= velocity <= 5;
            velocity_guess - 0.5 <= velocity <= velocity_guess + 0.5;
            angular_rate_guess - 0.5 <= angular_rate_cvx <= angular_rate_guess + 0.5;
            -2.2 * time_step <= angular_rate_cvx(2:num_points) - angular_rate_cvx(1:(num_points-1)) <= 2.2 * time_step;
            -2.2 * time_step <= velocity(2:num_points) - velocity(1:(num_points-1)) <= 2.2 * time_step;
            -0.05 <= angular_rate_cvx(1) <= 0.05;
            -0.05 <= angular_rate_cvx(num_points) <= 0.05;
    cvx_end
    
    % Update angular rate and velocity guess with optimization results
    velocity_guess = velocity;
    angular_rate_guess = angular_rate_cvx;
    
    % Output optimization value
    cvx_optval
    
    % Decrement iteration counter
    iteration_counter = iteration_counter - 1;

    % Plot path after each optimization iteration
    [x_path, y_path] = plot_karo(start_x, start_y, goal_x, goal_y, velocity_guess, angular_rate_guess, start_theta, time_step, num_points);
end
