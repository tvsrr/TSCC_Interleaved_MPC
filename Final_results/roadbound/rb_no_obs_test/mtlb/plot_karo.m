    %plotting 
function [x1,y1,xob,yob]=plot_karo(x0,y0,x_in,y_in,v_guess,w_guess,theta_in,delt,x_points_left,y_points_left,x_points_right,y_points_right,wp,red,n,lo,x_ob,y_ob,r_ob,roadx,roady,road_bound,obavoid,n_ob,v_movx,v_movy,obroadx,obroady)
init=theta_in;
 x1(1)=x0+v_guess(1)*delt*(cos(theta_in+w_guess(1)*delt));
 theta_in=theta_in+w_guess(1)*delt;
for i=2:n
    x1(i)=(x1(i-1)+v_guess(i)*delt*(cos(theta_in+w_guess(i)*delt)));
    if i~=n
    theta_in=theta_in+(w_guess(i)*delt);
    end
end
theta_in=init;
y1(1)=y0+v_guess(1)*delt*(sin(theta_in+w_guess(1)*delt));
 theta_in=theta_in+w_guess(1)*delt;
for i=2:n
    y1(i)=(y1(i-1)+v_guess(i)*delt*(sin(theta_in+w_guess(i)*delt)));
    if i~=n
    theta_in=theta_in+(w_guess(i)*delt);
    end
end
%clf;
for tick = 1:size(x_ob,2)
xob(tick,:) = x_ob(tick)+(1:n_ob)*v_movx(tick)*cos(0);
yob(tick,:) = y_ob(tick)+(1:n_ob)*v_movy(tick)*sin(0);
end

% xob2 = x_ob(2)+(1:n_ob)*v_movx(2)*cos(0);
% yob2 = y_ob(2)+(1:n_ob)*v_movy(2)*sin(0);
% 
% xob3 = x_ob(3)+(1:n_ob)*v_movx(3)*cos(0);
% yob3 = y_ob(3)+(1:n_ob)*v_movy(3)*sin(0);
%{
for ri=1:size(roadx,2)
plot(x1(1:red),y1(1:red),'b.', 'markersize',10);
hold on
 plot(x1,y1,'b.', 'markersize',5);
 plot(x_in,y_in,'g.', 'markersize',10);
if (road_bound~=0)

plot(x_points_left,y_points_left,'k','linewidth',2);
clx=(x_points_left+x_points_right)/2;cly=(y_points_left+y_points_right)/2;
plot(clx,cly,'k--');
plot(x_points_right,y_points_right,'k','linewidth',2);

end

if(obavoid~=0)
    for tick = 1:size(x_ob,2)
filledCircle([obroadx{tick}(ri),obroady{tick}(ri)],r_ob(1),1000,'r');
    end     
end

plot(lo(:,1),lo(:,2),'g.', 'markersize',10);

if (ri==size(roadx,2))
    roadx=[roadx,x1(1:10)];
   roady=[roady,y1(1:10)];
end
% motion ka emotion
plot(roadx,roady);
filledCircle([roadx(ri),roady(ri)],r_ob(1),1000,'b');
%axis equal
pause(0.001);
if(ri~=size(roadx,2))
cla;    
end
end
%}
end
