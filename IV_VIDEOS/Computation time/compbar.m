%computation time plotter 

x1 = 10.52;
x2 = 24.1371;
x3 = 51.651;

y1=8.436;
y2=10.97;
y3=20.765;

y=[x1,y1;x2,y2;x3,y3];
c=[50;100;150];
j=bar(c,y);
% set(gca,'xtick',[])
h = legend('MPC','TSCC coupled MPC');
ylabel=('Run Time[s]');
xlabel=('N[steps]');
set(h,'FontSize',30);
barvalues(j,2);