%bargraph maker 

d11 = 1.4; d21 = 0.8;
d12 = 3.2; d22 = 1.23;


y = [d11,d21;d12,d22];
c = categorical({'Obstacle avoidance';'Lane merging'});
bar(c,y);

h = legend('MPC','Our Framework');
ylabel('Mean deviation from initial path[m]','fontsize',40);
set(h,'FontSize',40);

