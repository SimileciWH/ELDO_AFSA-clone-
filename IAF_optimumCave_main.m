%%
%在IAF算法的基础上，将曲线进一步优化

%%
% close all;
clear all;
clc;
%%
%画出机器人的障碍环境，并标注start,goal位置
% a = rand(20)>0.35;%黑白格所占比例
tic;

a = load('environment.txt');%blacb--barrier occputy 35%.
% a = rand(20)>0.3;
n = size(a,1);
b = a;
b(end+1,end+1) = 0;
figure;
colormap([0 0 0;1 1 1])
pcolor(b); % 赋予栅格颜色
set(gca,'XTick',[10 20],'YTick',[10 20]);  % 设置坐标
axis image xy

displayNum(n);%显示栅格中的数值

text(1,21.5,'START','Color','red','FontSize',10);%显示start字符
text(21,1.5,'GOAL','Color','red','FontSize',10);%显示goal字符

hold on
%pin strat goal positon
scatter(1+0.5,20+0.5,'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0], 'LineWidth',1);%start point
scatter(20+0.5,1+0.5,'MarkerEdgeColor',[0 1 0],'MarkerFaceColor',[0 1 0], 'LineWidth',1);%goal point

hold on

%%
%障碍空间矩阵集合B
Barrier = (find(a==0))';

%%
%建立可行域

%%
%main
%%
% arrayValue = [13,34,256];
% %判断解是否处于障碍物中
% NB = length(arrayValue);
% for i = 1:1:NB
%     if ismember(arrayValue(i),Barrier) ==1
%         arrayValue(i) = randi([1 400],1); %删除出现在障碍物处的坐标.并赋予1-400中随机整数 
%     end
% end
N = 50;%人工鱼数量
try_number = 8;
MAXGEN = 100;%最大迭代次数
visual = 10;
delta = 0.618;
start = 20;%人工鱼群开始位置
DistMin = sqrt((1-n)^2+(n-1)^2);%最短距离标志位
goal = 381;%目标位置
shift = 1;%觅食行为和{群聚，追尾}行为的模式切换，
shiftFreq = 5;%觅食行为和{群聚，追尾}行为的切换频率
rightInf = sqrt(2);
% arrayValue = [20 39 58 77 96 115 134 153 172 192 212 232 252 272 291 310 
%                 329 328 327 326 325 324 344 364 363 362 361 381];%初始解
arrayValue = [20];%起点先放入要显示的向量中.
for i =1:1:N
    ppValue(i) = start;%传递人工鱼群位置的变量
end
H =GrideAF_foodconsistence(n,ppValue,goal);
count = 1;%记录执行觅食行为的次数
runDist = 0;%记录行走路径的总长度
runDist_part = 0;%记录每一条可行的行走路径长度，用于比较
BestH = zeros(1,MAXGEN);%记录每一次迭代中的最优H值 
index = [];%记录找到路径的鱼群
Q = 3;%为待优化指数
KQ = 3;%待优化搜索参数
optimum_dis = 0;%参数优化中，累计单个可行域中可行解之间的距离
iop = 1;%优化中的循环变量
%-----------------------------------------至此变量参数初始化全部结束------------------------------------------
for j = 1:1:MAXGEN
    switch shift
%------------------------------------觅食行为-----------------------------------------------------------------
        case 1
            for i = 1:1:N
               [nextPosition,nextPositionH] = GridAF_prey(n,ppValue(i),i,try_number,H,Barrier,goal);
               %需要记录下每条鱼对应的位置，以及食物浓度.以便下次更新.
               position(j,i) = nextPosition;%position存放所有鱼群的位置.
               H(i) = nextPositionH;
            end
            disp('prey!!!!')
%------------------------------------群聚行为---------------------------------------------------------------------------    
         case 2     
            for i = 1:1:N
                [nextPosition_S,nextPositionH_S] = GridAF_swarm(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal);
                [nextPosition_F,nextPositionH_F] = GridAF_follow(n,N,position(j-1,:),i,visual,delta,try_number,H,Barrier,goal);
                if nextPositionH_F < nextPositionH_S
                    nextPosition = nextPosition_F;
                    nextPositionH = nextPositionH_F;
                else
                    nextPosition = nextPosition_S;
                    nextPositionH = nextPositionH_S;
                end
                 position(j,i) = nextPosition;%position存放所有鱼群的位置.
                 H(i) = nextPositionH;
            end 
            disp('swarm & follow!!!')
    end
%-----------------------------------------------------------------------------------------------------------------
    count = count+1;%所有人工鱼都完成了一次觅食行为
    if rem(count,shiftFreq) == 0 %因为count从1开始记，所以5时，正好4次觅食行为
        shift = 2;
    else
        shift = 1;
    end
    
    %要更新ppValue的值
    ppValue =  position(j,:);
    %当在position中找到人工鱼到达goal处时，则跳出循环
    index = find(position(j,:)==goal);
    if ~isempty(index)
        break;
    end

end

%%如果是因为最大迭代次数而结束的循环，则说明没有路径到达
if MAXGEN <= j
    disp('There is no way can arrive to the goal!!!');
else
    
%%
%在所有可行路径中找出最短路径
for i = 1:1:length(index)
    arrayValue = [start;position(:,index(i))]';
%----------------------计算出行走路径的总长度-----------------------------
    for j = 1:1:length(arrayValue)-1
        d = distance(n,arrayValue(j),arrayValue(j+1));
        runDist_part = runDist_part + d;
    end
    transimit(i) = runDist_part;%记录所有可行路径的总长度    
end
[runDist,runMin_index] = min(transimit);
arrayValue = [start;position(:,index(runMin_index))]';

for i =1:1:length(arrayValue)
    BestH(i) = goal-arrayValue(i);%记录最优可行解的迭代图
end
%%
% %优化所有可行的路径
test = arrayValue;

% while(1)
%    
%     while iop < length(arrayValue) 
%         allow_area_now = allow_fun(n,Barrier,arrayValue(iop),KQ*rightInf );%得出当前位置人工鱼的可行域
%         [cin,ia,ib] = intersect(allow_area_now,arrayValue);%找出两个矩阵的公共元素，ia为公共元素在第一个矩阵中的位置，ib类似

%         if length(cin) >= Q%说明需要优化
%             for k = 1:1:length(cin)-2%减二是为了去除上一个点和本身
%                 optimum_dis = optimum_dis + distance(n,arrayValue(iop+k-1),arrayValue(iop+k));
%                 disp("判断是否优化！")
%             end
%             %将不符合要求的点清0，这样不会改变矩阵的维数
%             if distance(n,arrayValue(iop),arrayValue(iop+k)) < optimum_dis
%                 for k1 = iop+1:1:iop+k-1
%                     arrayValue(k1) = [];
%                     iop = 1;%因为arrayValue矩阵维数变化，要重新循环,iop要置1
%                     disp("清除不优解！")
%                     break;
%                 end
%             end
%         end
%         iop = iop+1;
%     end
%     if iop >= length(arrayValue)%判断上面的循环是否是正常跳出，如果是，那么说明优化全部结束.
%         break;
%     end
% end
%%
    DrawPath(n,arrayValue);%画出行走路径       
%------------------------------------------------------------------------
    fprintf('行走长度为: %f\n',runDist)%显示行走路径的长度
end
%-----------------------------------------------------------------------
%%画出迭代图
figure
plot(1:MAXGEN,BestH)
xlabel('迭代次数')
ylabel('优化值')
title('鱼群算法迭代过程')
%----------------------------------------------------------------------
%%
toc;
% PS:数值后面加0.5是为了保证，点居于坐标轴栅格中心处