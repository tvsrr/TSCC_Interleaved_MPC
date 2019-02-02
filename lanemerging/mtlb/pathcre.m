%pathcreator 
bx=0:20;by=0:20;
bbx = 20:50;
bby = 20*ones(1,size((20:50),2));
lbbx = 0:50;
lbby = 20*(ones(1,size((20:50),2)))+5;
rbbx = 20:50;
rbby = 20*(ones(1,size((20:50),2)))-5;
plot(bx,by);
hold on 
plot(bbx,bby);
plot(lbbx);
plot(zeros(1,size(lbby,2)),lbby);
plot(rbbx,rbby);