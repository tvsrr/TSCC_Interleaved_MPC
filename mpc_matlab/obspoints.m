%dynamic obstacle making
%This particular function is used to opbtain obstacle points with respect
%to time instants.
%The idea is vehicle obstacle constraint covers individual locations of the
%obstacle and the vehicle at that particular instant only

function [xpts,ypts,rpts]=obspoints(x_ob,y_ob,r_ob,n_ob,v_obx,v_oby,delt)
xpts=[];ypts=[];rpts=[];
for i=1:size(x_ob,2)
   
    x0=x_ob(i);y0=y_ob(i);r=r_ob(i)*ones(1,n_ob);
    obx_pts = x0+[1:n_ob]*v_obx(i)*cos(0);
    oby_pts = y0+[1:n_ob]*v_oby(i)*sin(0);
       xpts=[xpts,obx_pts];
    ypts=[ypts,oby_pts];
    rpts=[rpts,r];
end
end











