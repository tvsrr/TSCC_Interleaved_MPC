%obstacle avoidance
function [w_ob,v_ob,obc,xpts,ypts,rpts]=obst_avoid(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,n,x_ob,y_ob,r_ob,x_cod,y_cod,n_ob,v_movx,v_movy,delt,red,xpts,ypts)
 
if(n<=n_ob)
        ola=n;
    else
        ola=n_ob;
end
linobst_v=zeros(ola,ola); 
linobst_w=zeros(ola,ola);
w_ob=[];v_ob=[];obc=[];
[xpts,ypts,rpts]=obspoints(x_ob,y_ob,r_ob,ola,v_movx,v_movy,delt);
    kam=size(x_ob,2);
    in=1;fin=ola;
    while(kam~=0)
    radm=rpts(in:fin);
    ob_x=xpts(in:fin);
    ob_y=ypts(in:fin);
    funval=((ob_x-x_cod(1:ola)).^2+(ob_y-y_cod(1:ola)).^2)-(radm+0.9).^2;
    for k=1:ola
    linobst_w(k,1:k)=(-2*(ob_x(k)-x_cod(k)))* coeffs_fin_x{k}+(-2*(ob_y(k)-y_cod(k)))* coeffs_fin_y{k};
    linobst_v(k,1:k)=(-2*(ob_x(k)-x_cod(k)))*coeffs_vel_x{k}+(-2*(ob_y(k)-y_cod(k)))*coeffs_vel_y{k};
    obstconst(k)=funval(k)+(-2*(ob_x(k)-x_cod(k)))*(const_x(k)-x_cod(k))+(-2*(ob_y(k)-y_cod(k)))*(const_y(k)-y_cod(k));
    end
    w_ob=[w_ob;linobst_w];
    v_ob=[v_ob;linobst_v];
    obc=[obc,obstconst];
    in=fin+1;fin=fin+ola;
    kam=kam-1;
    end
end
%correct this
%Am i avoiding obstacles at the same stepcount or not?
%How do i dispaly obstacles at the correct time they go?
