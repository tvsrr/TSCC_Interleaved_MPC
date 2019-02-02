function [A_coll,Q_coll,C_coll,n_pass,can,x_pass,y_pass]=wpcoffmaker(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,n,n_w,wp)
%---------------------------------------------------------------------%
%steps to write in mpc
% consider the waypoints
% give the n values
% replace the single goal point with the set of values
% reduce the n values at every instant
% remove the <0 parts
% add the >0 parts
%----------------------------------------------------------------------%
can = find(n_w<=n )';
for i = 1:size(can,2)
x_pass(i)= wp(can(i),1);
y_pass(i)=wp(can(i),2);
n_pass(i)=n_w(can(i));
end
A_coll = cell(size(n_pass,2),1);
Q_coll = cell(size(n_pass,2),1);
%A,Q,C set for the waypoints
for i = 1:size(n_pass,2)
[A,Q,C]= mat_creator(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,n_pass(i),x_pass(i),y_pass(i));
A_coll{i} =A;
Q_coll{i} =Q;
C_coll(i)=C;
end
end