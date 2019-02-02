%trackgeneration 
x = 2429.5; y = 229.531;
xend = 2429.5; yend = 26.2902;
xc = (x+xend)/2; yc = (y+yend)/2;
xa =[]; ya=[]; xa2 =[]; ya2 =[]; xcod=[];ycod=[];xcod2=[];ycod2=[];
la=[];la2=[];ld=[];ld2=[];
for t = 90:270
xa = [xa,xc-100*cosd(t)];
ya = [ya,yc-100*sind(t)];
la = [la,100*t*pi/180];
end
for i = 1:2400
    xcod = [xcod,xend-i];
    ld =[ld,la(end)+i ];
end
ycod = yend*ones(1,size(xcod,2));
chrd = pdist([x,y;xend,yend]);
xnew = xcod(end);
ynew = ycod(end)+chrd;
xc2= (xnew+xcod(end))/2; yc2 = (ynew+ycod(end))/2;
for t = 90:270
xa2 = [xa2,xc2+100*cosd(t)];
ya2 = [ya2,yc2+100*sind(t)];
la2 = [la2,100*t*pi/180];
end
for i = 1:2400 
    xcod2=[xcod2,xnew+i];
    ld2 = [ld2,la2(end)+i];
end
ycod2 = ynew*ones(1,size(xcod2,2));

finx = [xa,xcod,xa2,xcod2];
finy = [ya,ycod,ya2,ycod2];
larc = [la,ld,la2,ld2]-la(1);

%plotting
% plot(xa,ya);
% hold on 
% plot(xcod,ycod);
% plot(xa2,ya2);
% plot(xcod2,ycod2);
% hold off 
%plot(finx,finy,'*');
pat = [finx',finy',larc'];
csvwrite('calc.csv',pat);
axis equal
