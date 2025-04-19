% Configuration parameters as a script 
config = struct();

config.simulation.dt = 0.2;
config.simulation.predictionHorizonSteps = 60; 
config.simulation.w_guess = 0.1*ones(1,config.simulation.predictionHorizonSteps );
config.simulation.v_guess = 5*ones(1,config.simulation.predictionHorizonSteps );
config.simulation.theta_initial = 0;
config.simulation.theta_final = 0;

config.robot.maxLinearVel = 2.0;
config.robot.AverageSpeed = 10.0;

config.obstacle.theta_obs = [0,0,0,0];
config.obstacle.radius = [1.5,2,1.5,0.5];
config.obstacle.Vx = [0.4,0.5,0.2];
config.obstacle.Vy=[0,0,0];
config.obstacle.num_wp_obstacle = 1000;

config.robot.LF = 1.27; 
config.robot.LR = 1.37;

