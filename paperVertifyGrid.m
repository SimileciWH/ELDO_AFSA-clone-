clc;
close all;
clear all;

a = [1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 0 1 1 1 1 1 1  ;
     1 1 1 0 0 0 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     1 1 1 1 1 1 1 1 1 1  ;
     ];
n = size(a,1);
b = a;
b(end+1,end+1) = 0;
figure;
colormap([0 0 0;1 1 1])
pcolor(b); % 赋予栅格颜色
set(gca,'XTick',2:2:10,'YTick',2:2:10);  % 设置坐标
axis image xy

% displayNum(n);%显示栅格中的数值.

hold on

scatter(7+0.5,4+0.5,'MarkerEdgeColor',[0 0 1],'LineWidth',1.5);%在栅格图中表示出来
text(7,4.8,'GOAL','Color','red','FontSize',8);%显示goal字符

% text(2.3,9.4,'w4','Color','m','FontSize',12);
text(3.3,8.4,'w3','Color','m','FontSize',12);
text(4.3,8.4,'w2','Color','m','FontSize',12);
text(5.3,8.4,'w1','Color','m','FontSize',12);

% DrawPath(n,[10 19,28,38,48]);
DrawPath(n,[9,18,27,26,35,44,54,64]);

hold on 
text(1,10.8,'Pbest','Color','blue','FontSize',10);
text(1,9.8,'Pb','Color','blue','FontSize',10);

text(2,8.8,'Pb(rb-1)','Color','blue','FontSize',10);
text(3,7.8,'Pb(rb)','Color','blue','FontSize',10);
text(3,6.2,'Pb(rb+1)','Color','blue','FontSize',10);

text(3,8.8,'Pbest(r-1)','Color','blue','FontSize',10);
text(4,8.2,'Pbest(r)','Color','blue','FontSize',10);
text(5,8.8,'Pbest(r+1)','Color','blue','FontSize',10);