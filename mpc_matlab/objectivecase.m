%path creation 
x1m = zeros(1,21);y1m=1:21;
x2m = 1+-1*cosd(1:90);%right turn main
y2m = 20+20*sind(1:90);
x3m = 1:40;
y3m = 40*ones(1,size(x3m,2));
x4=x2m+4;y4=y2m-4;%right turn bound
x5=-x2m-4;y5=y2m-4;%lrft turn bound
x6=x1m-4;y6=y1m;
x7=x1m+4;y7=y1m;
x8=x5(end):-1:-15; %back turn 
y8=36*ones(1,size(x8,2));
x9=x4(end):40;y9 = 36*ones(1,size(x9,2));
x10=-15:40;y10=45*ones(1,size(x10,2));
%% summarizing 
xbot = [x1m,x2m,x3m];ybot = [y1m,y2m,y3m];
xbound=[x4,x5,x6,x7,x8,x9,x10]; ybound=[y4,y5,y6,y7,y8,y9,y10];
%%
% plot(x1m,y1m);
% hold on 
% plot(x2m,y2m);
% plot(x3m,y3m);

for i= 1:size(xbot,2)
x= xbot(i)-1; y = ybot(i);
r=rectangle('position',[x-.75 y-2.5 3 5],'curvature',0.2,'FaceColor','r');
rotate(r,y)
hold on 
plot(xbot,ybot,'k');
plot(x4,y4,'k');
plot(x5,y5,'k');
plot(x6,y6,'k');
plot(x7,y7,'k');
plot(x8,y8,'k');
plot(x9,y9,'k');
plot(x10,y10,'k');
axis equal
pause(0.05)
cla
end
