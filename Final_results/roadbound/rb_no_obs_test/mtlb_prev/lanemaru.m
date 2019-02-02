function [wp]= lanemaru(wp,x_ob)
x_c = wp(:,1);
%car is 4m wide and 5m long
for i=1:size(x_ob,2)
   nw = find((x_ob(i)-7<x_c));
   wp(nw(1):end,2)=wp(nw(1):end,2)+10; 
end