function [d1,d2]=calculatedeviation(cumx,cumy,tumx,tumy)
pat = load('mypath2.csv');
px = pat(2:61,:);
py = zeros(1,60);
for i = 1:60
dist1(i) = sqrt((cumx(i)-px(i))^2+(cumy(i)-py(i))^2);
dist2(i) = sqrt((tumx(i)-px(i))^2+(tumy(i)-py(i))^2);
end

d1 = mean(dist1);
d2 = mean(dist2);

end