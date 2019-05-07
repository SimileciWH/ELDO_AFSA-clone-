close all;
clear all;
clc;
%%
%画出机器人的障碍环境，并标注start,goal位置
% a = rand(20)>0.35;%黑白格所占比例
tic;

a = load('environment.txt');%blacb--barrier occputy 35%.
n = size(a,1);
b = a;
b(end+1,end+1) = 0;
figure(1);
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
N = 20;%人工鱼数量
try_number = 8;
MAXGEN = 100;%最大迭代次数
visual = 6;
delta = 0.618;
start = 20;%人工鱼群开始位置
goal_false = 189;%栅格图中心附近的任意一点，在可行域中
goal_true = 381;%人工鱼目标位置,第二组鱼群
start_2 = goal_true;
DistMin = sqrt((1-n)^2+(n-1)^2);%最短距离标志位
DistMin2 = sqrt((1-n)^2+(n-1)^2);
shift = 1;%觅食行为和{群聚，追尾}行为的模式切换，
shiftFreq = 5;%觅食行为和{群聚，追尾}行为的切换频率
countMin = 2;%记录最优解个数.
countMin2 = 2;
rightInf = sqrt(2);
% arrayValue = [20 39 58 77 96 115 134 153 172 192 212 232 252 272 291 310 
%                 329 328 327 326 325 324 344 364 363 362 361 381];%初始解
arrayValue1 = [20];%起点先放入要显示的向量中.
arrayValue2 = [381];
for i =1:1:N
    ppValue(i) = start;%传递人工鱼群位置的变量
end

for i =1:1:N
    ppValue2(i) = start_2;%传递人工鱼群位置的变量
end
H = GrideAF_foodconsistence(n,ppValue,goal_false);

H2 = GrideAF_foodconsistence(n,ppValue2,goal_false);
count = 1;%记录执行觅食行为的次数
runDist = 0;%记录行走路径的总长度
BestH1 = zeros(1,MAXGEN);%记录每一次迭代中的最优H值 
BestH2 = zeros(1,MAXGEN);
%-----------------------------------------至此变量参数初始化全部结束------------------------------------------------------
for j = 1:1:MAXGEN
    switch shift
%------------------------------------觅食行为----------------------------------------------------------------------------
        case 1
            for i = 1:1:N
               [nextPosition_1,nextPositionH_1] = GridAF_prey(n,ppValue(i),i,try_number,H,Barrier,goal_false);
               
                [nextPosition_2,nextPositionH_2] = GridAF_prey(n,ppValue2(i),i,try_number,H,Barrier,goal_false);
               %需要记录下每条鱼对应的位置，以及食物浓度.以便下次更新.
               position(i) = nextPosition_1;%position存放所有鱼群的位置.
               H(i) = nextPositionH_1;
               
               position2(i) = nextPosition_2;%position存放所有鱼群的位置.
               H2(i) = nextPositionH_2;
            end
            disp('prey!!!!')
%------------------------------------群聚行为---------------------------------------------------------------------------    
         case 2     
            for i = 1:1:N
                [nextPosition_S_1,nextPositionH_S_1] = GridAF_swarm(n,N,position,i,visual,delta,try_number,H,Barrier,goal_false);
                [nextPosition_F_1,nextPositionH_F_1] = GridAF_follow(n,N,position,i,visual,delta,try_number,H,Barrier,goal_false);
                
                [nextPosition_S_2,nextPositionH_S_2] = GridAF_swarm(n,N,position2,i,visual,delta,try_number,H2,Barrier,goal_false);
                [nextPosition_F_2,nextPositionH_F_2] = GridAF_follow(n,N,position2,i,visual,delta,try_number,H2,Barrier,goal_false);
                
                if nextPositionH_F_1 < nextPositionH_S_1
                    nextPosition_1 = nextPosition_F_1;
                    nextPositionH_1 = nextPositionH_F_1;
                else
                    nextPosition_1 = nextPosition_S_1;
                    nextPositionH_1 = nextPositionH_S_1;
                end
                
                if nextPositionH_F_2 < nextPositionH_S_2
                    nextPosition_2 = nextPosition_F_2;
                    nextPositionH_2 = nextPositionH_F_2;
                else
                    nextPosition_2 = nextPosition_S_2;
                    nextPositionH_2 = nextPositionH_S_2;
                end
                
                 position(i) = nextPosition_1;%position存放所有鱼群的位置.
                 H(i) = nextPositionH_1;
                 
                 position2(i) = nextPosition_2;%position存放所有鱼群的位置.
                 H2(i) = nextPositionH_2;
                 
            end 
            disp('swarm & follow!!!')
    end
%-----------------------------------------------------------------------------------------------------------------
    count = count+1;%所有人工鱼都完成了一次行为
    if rem(count,shiftFreq) == 0 %因为count从1开始记，所以5时，正好4次觅食行为
        shift = 2;
    else
        shift = 1;
    end
%================================================================================================================
    [Hmin,index] = min(H);%找到Hmin对应的杀个位置.存在index中  
    %要更新ppValue的值
    ppValue =  position;
    %更新最小值到arrayValue中去
    
    [H2min,index2] = min(H2);%找到Hmin对应的杀个位置.存在index中  
    %要更新ppValue的值
    ppValue2 =  position2;
    %更新最小值到arrayValue中去
    
    if Hmin < DistMin
        DistMin = Hmin;
        arrayValue1(countMin) = position(index);%把每一步中最优的解记录下来  
    else
        DistMin = DistMin;
        arrayValue1(countMin) = arrayValue1(countMin-1);
    end
    
    if H2min < DistMin2
        DistMin2 = H2min;
        arrayValue2(countMin2) = position2(index2);%把每一步中最优的解记录下来  
    else
        DistMin2 = DistMin2;
        arrayValue2(countMin2) = arrayValue2(countMin2-1);
    end
    
    BestH1(j) = DistMin;%记录每一次迭代中的最优H值 
    countMin = countMin+1;
    
    BestH2(j) = DistMin2;%记录每一次迭代中的最优H值 
    countMin2 = countMin2+1;
    
    if arrayValue1(countMin-1) == goal_false && arrayValue2(countMin2-1) == goal_false
       break;
    end
    
% %为去除局部最优解对全局的影响，判断arrayValue中每个值是否在上一个值得可行域内
%     if distance(n,arrayValue(countMin-1),arrayValue(countMin-2)) > rightInf
%          [nextPosition,nextPositionH] = GridAF_prey(n,arrayValue(countMin-2),index,try_number,H,Barrier,goal);
%          arrayValue(countMin-1) = nextPosition;
%     end
end
%%
%把arrayValue1和arrayValue2整合成arrayValue
arrayValue = [arrayValue1 arrayValue2(end:-1:1)];

%%如果是因为最大迭代次数而结束的循环，则说明没有路径到达
if MAXGEN <= j
    disp('There is no way can arrive to the goal!!!');
else
    DrawPath(n,arrayValue);%画出行走路径   
    
%----------------------计算出行走路径的总长度-----------------------------
    for i = 1:1:length(arrayValue)-1
        d = distance(n,arrayValue(i),arrayValue(i+1));
        runDist = runDist + d;
    end
%------------------------------------------------------------------------
    fprintf('行走长度为: %f\n',runDist)%显示行走路径的长度
end
%-----------------------------------------------------------------------
%%画出迭代图
% figure
% plot(1:MAXGEN,BestH1)
% xlabel('迭代次数')
% ylabel('优化值')
% title('鱼群算法迭代过程1')
% 
% figure
% plot(1:MAXGEN,BestH2)
% xlabel('迭代次数')
% ylabel('优化值')
% title('鱼群算法迭代过程2')
%----------------------------------------------------------------------
%%
toc;
% PS:数值后面加0.5是为了保证，点居于坐标轴栅格中心处