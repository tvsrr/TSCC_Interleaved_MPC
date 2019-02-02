%AQC matrix construction 
function [A,Q,C]= mat_creator(coeffs_fin_x,coeffs_vel_x,coeffs_fin_y,coeffs_vel_y,const_x,const_y,num,xg,yg)
k1= coeffs_fin_x{num};
l1=coeffs_vel_x{num};
k = [k1,l1];
A= k'*k;
k2=coeffs_fin_y{num};
l2=coeffs_vel_y{num};
k = [k2,l2];
A = A+k'*k; %coefficients matrix has to be concat of w and v
n1=(const_x(num)-xg);
n2=(const_y(num)-yg);
Q = [2*(k1*n1+k2*n2),2*(l1*n1+l2*n2)];
C= n1^2+n2^2;
end