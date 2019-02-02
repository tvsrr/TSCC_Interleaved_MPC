function []=carmovement(X,Y)
for i=1:size(X,2)
plot(X,Y);
hold on
plot(X(1),Y(1),'m.','markersize',30);
plot(X(end),Y(end),'m.','markersize',30);
plot(X(i),Y(i),'b.','markersize',50);
pause(0.01);
if(i~=size(X,2))
cla;    
end
end
end
