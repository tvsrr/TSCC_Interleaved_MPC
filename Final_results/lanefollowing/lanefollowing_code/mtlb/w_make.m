%building the new w
function [coeffs_fin_x,coeffs_fin_y,const_x,const_y]=w_make(delt,n,w_guess,theta_in,v_guess,x0,y0)
z=exp(-1.16*delt); %change the time constant here
co_sys=z.^((1:n)-1);
co_sys=(1-z)*co_sys;
k_unit = triu(ones(n,n));
k_lnit = tril(ones(n,n));
mashalla=zeros(1,n);
for ah = 1:n
row= [zeros(1,ah-1),co_sys(1:n-(ah-1))];
mashalla= [mashalla;row];
end
mashalla(1,:)=[];
w_mat= w_guess*mashalla;
for i = 1:n
w_norm(i,:)= w_mat.*k_lnit(i,:);%format = [w1,0,0;w1,w1+w2,0;w1,w1+w2,w1+w2+w3]
w_new(i,:)= w_mat.*k_unit(i,:);%for linearization format = [w1,w1+w2,w1+w2+w3;0,w1+w2,w1+w2+w3;0,0,w1+w2+w3]
end
w_norm = theta_in*tril(ones(n,n))+w_norm*delt;  %All thetas arranged in the lower triangle format
w_x = tril(delt*cos(w_norm)); 
w_y = tril(delt*sin(w_norm)); 
%---for linearization purpose---%
w_new = theta_in*triu(ones(n,n))+w_new*delt;  %All thetas arranged in the upper triangle format
w_lin_x = triu(delt*cos(w_new)); 
w_lin_y = triu(delt*sin(w_new)); 
for i = 1:n
    velo(i,:) = v_guess.*(k_lnit(i,:));%format = [v1,0,0;v1,v2,0;v1,v2,v3]
    vllo(i,:) =  v_guess.*(k_unit(i,:));%for linearization format:[v1,v2,v3;0,v2,v3;0,0,v3]
end
x_ang = ( (velo .* w_x));
y_ang = ( (velo .* w_y));
x_lin_ang=( (vllo .* w_lin_x));
y_lin_ang = ( (vllo .* w_lin_y));
for i = 1:n
x_cod(i)=x0 + sum (x_ang(i,:)); % X- co ordinate ready with guess information
y_cod(i)=y0 + sum (y_ang(i,:)); %Y - co ordinate ready with guess information 
end
%% Generating mashalla matrix
z_set= z.^((1:n)-1);
z_mat= z_set*k_unit;
mashalla=zeros(1,n);
for ah = 1:n
row= [zeros(1,ah-1),z_mat(1:n-(ah-1))];
mashalla= [mashalla;row];
end
mashalla(1,:)=[];
%% Linearization in matrix form 
%w linearization
for i = 1:n
mat= -(1-z)*delt*(y_lin_ang(1:i,1:i)).*(mashalla(1:i,1:i));
% kal =  triu(ones(i,i));
coeffs_fin_x{i}=sum((mat),2)';
mat= -(1-z)*delt*(x_lin_ang(1:i,1:i)).*(mashalla(1:i,1:i));
coeffs_fin_y{i}= sum((mat),2)';

const_x(i) = x_cod(i) + coeffs_fin_x{i}*(-1*w_guess(1:i)');
const_y(i)= y_cod(i) + coeffs_fin_y{i}*(-1*w_guess(1:i)');
end

end
