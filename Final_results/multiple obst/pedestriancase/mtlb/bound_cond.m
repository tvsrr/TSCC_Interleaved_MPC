%boundary conditions
function [left_tangent,right_tangent]=bound_cond(x_even_left,x_even_right,x_cod_p,y_cod_p,y_even_left,y_even_right,arc_param_left,arc_param_right,coff_arc_x_right, coff_arc_y_right,coff_arc_x_left, coff_arc_y_left)
end_size=size(x_even_left,1);
dist_left = (sqrt((x_even_left-x_cod_p).^2+(y_even_left-y_cod_p).^2));
dist_right= (sqrt((x_even_right-x_cod_p).^2+(y_even_right-y_cod_p).^2));
[~,ind_left]= min(dist_left);
[~,ind_right]=min(dist_right);
if(ind_left ~= end_size)
k1 = ind_left+1;
left_x= coff_arc_x_left(k1-1,:);
left_y= coff_arc_y_left(k1-1,:);
m_left= linspace(arc_param_left(ind_left),arc_param_left(k1),100);
else
    k1= ind_left-1;
    left_x =coff_arc_x_left(k1,:);
    left_y = coff_arc_y_left(k1,:);
m_left= linspace(arc_param_left(k1),arc_param_left(ind_left),100);    
end
if(ind_right ~= end_size)
    k2 = ind_right+1; 
    right_x =coff_arc_x_right(k2-1,:); 
    right_y =coff_arc_y_right(k2-1,:);
m_right=linspace(arc_param_right(ind_right),arc_param_left(k2),100);
else
     k2=ind_right-1;
     right_x =coff_arc_x_right(k2,:);
     right_y = coff_arc_y_right(k2,:);
m_right=linspace(arc_param_right(k2),arc_param_left(ind_right),100);
end
%conditions to find the parameter
x_lp= left_x(1)*(m_left-arc_param_left(ind_left)).^3+left_x(2)*(m_left-arc_param_left(ind_left)).^2+left_x(3)*(m_left-arc_param_left(ind_left))+left_x(4);
y_lp=  left_y(1)*(m_left-arc_param_left(ind_left)).^3+left_y(2)*(m_left-arc_param_left(ind_left)).^2+left_y(3)*(m_left-arc_param_left(ind_left))+left_y(4);
x_rp= right_x(1)*(m_right-arc_param_right(ind_right)).^3+right_x(2)*(m_right-arc_param_right(ind_right)).^2+right_x(3)*(m_right-arc_param_right(ind_right))+right_x(4);
y_rp=  right_y(1)*(m_right-arc_param_right(ind_right)).^3+right_y(2)*(m_right-arc_param_right(ind_right)).^2+right_y(3)*(m_right-arc_param_right(ind_right))+right_y(4);
% finding the projection
dist_pl = (sqrt((x_lp-x_cod_p).^2+(y_lp-y_cod_p).^2)); %projection distance
dist_pr = (sqrt((x_rp-x_cod_p).^2+(y_rp-y_cod_p).^2));
[~,ind_pl]= min(dist_pl);
[~,ind_pr]= min(dist_pr);
slope_l= (3*left_y(1)*(m_left(ind_pl)-arc_param_left(ind_left))^2+2*left_y(2)*(m_left(ind_pl)-arc_param_left(ind_left))+left_y(3))/(3*left_x(1)*(m_left(ind_pl)-arc_param_left(ind_left))^2+2*left_x(2)*(m_left(ind_pl)-arc_param_left(ind_left))+left_x(3));
slope_r=(3*right_y(1)*(m_right(ind_pr)-arc_param_right(ind_right))^2+2*right_y(2)*(m_right(ind_pr)-arc_param_right(ind_right))+right_y(3))/ (3*right_x(1)*(m_right(ind_pr)-arc_param_right(ind_right))^2+2*right_x(2)*(m_right(ind_pr)-arc_param_right(ind_right))+right_x(3));
if (slope_l==Inf)
    slope_l=100000;
end
if(slope_r== Inf)
    slope_r=100000;
end
left_tangent = [-slope_l;1;(slope_l*x_lp(ind_pl)-y_lp(ind_pl))];
right_tangent = [-slope_r;1;(slope_r*x_rp(ind_pr)-y_rp(ind_pr))];

%find the slope:doubt is there are two curves 1 projection point 
%find the equation using the point slope form
%send the coefficients to the optimization as a constraint

end