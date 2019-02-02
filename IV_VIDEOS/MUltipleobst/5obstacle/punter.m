%plotter of velocities
vel = load('cum_vel_4obs.csv');
vel = smooth(vel);
figure();
plot(vel,'r','LineWidth',4);
axis([0 60 0 10]);
title('Linear Velocity profile','FontSize',40);
xlabel('number of time steps','FontSize',40);
ylabel('Velocity[m/s]','FontSize',40);
om = load('cum_om_4obs.csv');
om = smooth(om);
figure();
plot(om,'k','LineWidth',4);
axis([0 60 -0.6 1.0]);
title('Angular Velocity profile','FontSize',40);
xlabel('number of time steps','FontSize',40);
ylabel('Thetadot[rad/s]','FontSize',40);