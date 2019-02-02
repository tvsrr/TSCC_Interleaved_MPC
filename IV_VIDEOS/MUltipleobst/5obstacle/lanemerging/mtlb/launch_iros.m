function [vlast,wlast,vsend,wsend,v_movx,v_movy,stateRobo,stateObst,oth,r_ob]=launch_iros(x0,y0,x_ob,y_ob,theta_in,na,mela,ob,vpal,wpa)
%tolerance correction line 146
%launch file 
%contains all the necessary function- calls
%The dependant functions are linerisation of the values,finding A,Q,C
% Formulatng the cost functons 
%This document will have the cvx- optimization
%Rectangles will be a function
%Obstacle avoidance for 3 circles will be afunction 
%Plotting will be another function
firstval = 0;
x0=double(x0);y0=double(y0);x_ob=double(x_ob);y_ob=double(y_ob);
theta_in=double(theta_in);na = double(na); mela = double(mela); 
vpal= double(vpal);wpa=double(wpa);
disp(vpal)
tic
[w_guess,v_guess,delt,x_points_left,y_points_left,x_points_right,y_points_right,wp,n_w,n_ob,v_movx,v_movy,r_ob,n,avg_speed,oth] = get_input(x0,y0,firstval,vpal,wpa);
mred=0;
x_in=x0;obinx = x_ob;
y_in=y0;obiny = y_ob;
thet = theta_in;
roadx=[];roady=[];obroadx={};obroady={};
lo=wp(:,1:2);
road_bound=0;
obavoid = ob;
lt=zeros(50,3);
rt=zeros(50,3);
 red=n;
 %disp(n_w)
cit= n_w(end)/red;
x_i=x_ob;y_i=y_ob;
k_ob = n_ob;
vsend=[];wsend=[];
y1=y0;x1=x0;thf=theta_in;
while (cit)
ct=1;
% for obi= 1:size(y1,2)
%     a= ((y1(obi)-tole) <= y_ob);
%     b=(y_ob<=(y1(obi)+tole));
%     val = ~(a&&b);
%     if (val)
%     cit=0;ct=0;
%     if (x1==x0)  %change this in case of looped path 
%         disp('entered')
%         vel=0;w_cvx=theta_in;
%     end
%     break;
%     end
% end
preval=0;supre=99;
% if obavoid ==1
%     if (((x_ob-x0)<=10)&&((x_ob-x0)>0)&&mela)
%     wp=lanemaru(wp,x_ob);
%     disp('satis');
%     mela=0;
%     end
% end
while(ct)
[coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,x_cod,y_cod,const_x,const_y,w_x]= linearise_hol(n,x0,y0,w_guess,v_guess ,delt,theta_in);
num = n;
if(road_bound~=0)
%  [A,Q,C]= mat_creator(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,num,xg,yg);
%using the boundaries
[ coff_arc_x_left, coff_arc_y_left, arc_param_left, x_even_left, y_even_left, x_arc_points_left, y_arc_points_left ] = boundary_making( x_points_left, y_points_left, 4, 1 ) ;
[ coff_arc_x_right, coff_arc_y_right, arc_param_right, x_even_right, y_even_right, x_arc_points_right, y_arc_points_right ] = boundary_making( x_points_right, y_points_right, 4, 1 ) ;

for i = 1:n
x_cod_p=x_cod(i);
y_cod_p=y_cod(i);
[left_tangent,right_tangent]=bound_cond(x_even_left,x_even_right,x_cod_p,y_cod_p,y_even_left,y_even_right,arc_param_left,arc_param_right,coff_arc_x_right, coff_arc_y_right,coff_arc_x_left, coff_arc_y_left);
lt(i,:)=left_tangent';
rt(i,:)=right_tangent';
end
end
[A_coll,Q_coll,C_coll,n_pass,can,x_pass,y_pass]=wpcoffmaker(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,n,n_w,wp);

[w_ob,v_ob,obc,xpts,ypts,rpts]=obst_avoid(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,n,x_ob,y_ob,r_ob,x_cod,y_cod,n_ob,v_movx,v_movy,delt,red);

 if(n<=n_ob)
        ola=n;
    else
        ola=n_ob;
 end

 %% optimization
cvx_begin quiet
variables w_cvx(1,n) vel(1,n) slack1(1,n) slack2(1,n)
%% road bound
if (road_bound~=0)
%road boundary condition
for i = 1:10
x_cod_p=coeffs_fin_x{i}*w_cvx(1:i)'+coeffs_vel_x{i}*vel(1:i)'+const_x(i);
y_cod_p=coeffs_fin_y{i}*w_cvx(1:i)'+coeffs_vel_y{i}*vel(1:i)'+const_y(i);
lsum(i)=([x_cod_p,y_cod_p,1]*lt(i,:)')+slack1(i);
rsum(i)=-([x_cod_p,y_cod_p,1]*rt(i,:)')+slack2(i);
end
else
    slack1==0;
    slack2==0;
end
%%
for mal = 1:size(n_pass,2)
new= [w_cvx(1:n_pass(mal)),vel(1:n_pass(mal))];
fin(mal) = new*A_coll{mal}*new'+Q_coll{mal}*new'+C_coll(mal);
end
fin3=theta_in;
for ila = 1:red
    fin3=fin3+sum(w_cvx(ila))*delt;
end
ful= sum(fin)+0.5*(sum(slack1.^2)+sum(slack2.^2));
minimize(ful) 
subject to 
if (road_bound~=0)
lsum==0;
rsum==0;
-slack1<=0;
-slack2<=0;
end
-0.25<=w_cvx<=0.25;
0<=vel<=13;     
v_guess - 0.5 <= vel <= v_guess + 0.5;
w_guess-0.5<=w_cvx<=w_guess + 0.5;
-2.2*delt <= w_cvx(2:n) - w_cvx(1:(n-1))<= 2.2*delt;
-2.2*delt <= vel(2:n) - vel(1:(n-1))<= 2.2*delt;
disp(['v_guess',num2str(v_guess(1)),'vpal',num2str(vpal)])
-delt*2.2<=(vpal-vel(1))<=delt*2.2
-delt*2.2<=(wpa-w_cvx(1))<=delt*2.2

%v-0.5<=vel(1)<=v+0.5
%w-0.5<=w_cvx(1)<=w+0.5
if (x_in==firstval)
-0.05<= w_cvx(1)<=0.05;
-0.05<= w_cvx(n)<=0.05;
end

if(obavoid==1)
%----------------------------------------------------------------------%
%obstacle avoidance
w_set=repmat(w_cvx(1:ola),ola,1);
w_set=tril(w_set);
w_set=repmat(w_set,size(x_ob,2),1);
v_set=repmat(vel(1:ola),ola,1);
v_set=tril(v_set);
v_set=repmat(v_set,size(x_ob,2),1);
-(sum(w_ob.*w_set,2)+sum(v_ob.*v_set,2)+obc')<=0;
end
%-----------------------------------------------------------------------%

cvx_end

clear fin lsum rsum slack1 slack2
cvx_optval
v_guess = vel;
w_guess = w_cvx;
ct = ct-1;
[x1,y1,xob,yob]=plot_karo(x0,y0,x_in,y_in,v_guess,w_guess,theta_in,delt,x_points_left,y_points_left,x_points_right,y_points_right,wp,red,n,lo,x_ob,y_ob,r_ob,roadx,roady,road_bound,obavoid,n_ob,v_movx,v_movy,obroadx,obroady);

if (abs(preval-cvx_optval)<=1 )  %tolerance corection 
    ct=0;
else
   ct=ct+1;
end

preval=cvx_optval;
end
n_w=n_w-red;
vsend=[vsend,vel];
wsend=[wsend,w_cvx];
if((x1~=x0)&(size(x1,2)~=1))
x0=x1(red);
y0=y1(red);
theta_in= theta_in+sum(w_guess(1:red))*delt;
%disp('size of n is ')
%disp(n)
%disp('size of n_w(end) is')
%disp(n_w(end))
if(n_w(end)<n && n_w(end)~=0)
    w_guess(1:(n-red))=w_guess(red+1:n);
    v_guess(1:(n-red))=v_guess(red+1:n);
    ck=n-red;
    n=n_w(end);
    w_guess(ck+1:n)=w_guess(ck)*ones(1,n-ck);
    v_guess(ck+1:n)=v_guess(ck)*ones(1,n-ck);
    w_guess(n+1:end)=[];
    v_guess(n+1:end)=[];
end
end
wp(:,3)=wp(:,3)-red;

for tick = 1:size(x_ob,2)
x_ob(tick)=xob(tick,red);
y_ob(tick)=yob(tick,red);
end

n_ob=n_ob-red;
if(size(w_guess,2)~=0 && n_w(end)>n)
%disp(n)
%disp(n_w(end))
w_up=w_guess(n);v_up=v_guess(n);
w_guess(1:(n-red))=w_guess(red+1:n);
w_guess((n-red+1):n)=w_up(ones(1,(red)));
v_guess(1:(n-red))=v_guess(red+1:n);
v_guess((n-red+1):n)=v_up(ones(1,(red)));
end
%disp('wguess is of size')
%disp(size(w_guess))
%csvwrite('v_guess.csv',v_guess);
%csvwrite('w_guess.csv',w_guess);
posit = find(n_w<=0);
n_w(posit)=[];
wp(posit,:)=[];
roadx=[roadx,x1(1:red)];
roady=[roady,y1(1:red)];
disp(n_w)
if size(obroadx,2)==0
for tick =1:size(x_ob,2)
obroadx{tick}=xob(tick,(1:red));
obroady{tick}=yob(tick,(1:red));
end
else
    for tick =1:size(x_ob,2)
    obroadx{tick}=[obroadx{tick},xob(tick,(1:red))];
    obroady{tick}=[obroady{tick},yob(tick,(1:red))];
    end
end
mred = mred+red;
if (mred>=na)
    break
end

cit=cit-1;
% end
end

lf = 1.27; lr = 1.37;
gr = abs(avg_speed*(sqrt(1-(lr*wsend/avg_speed).^2)));
stesend = atan((wsend*(lf+lr))./gr );

vlast = vsend(end);wlast=wsend(end);
if (x_in==0)
%vsend=[0.01,vsend];
%wsend=[0.01,wsend];
end
stateRobo=[x_in,y_in,thet,vpal,wpa];
for obi= 1:size(obinx,2)
stateObst(obi,:)=[obinx(obi),obiny(obi),oth(obi),v_movx(obi)];
end

%csvwrite('vec.csv',vsend);
%csvwrite('woc.csv',stesend);
end 