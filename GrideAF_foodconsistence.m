function [ H ] = GrideAF_foodconsistence(n,X,goal )
%输入参数：
% n----矩阵维数
% X----人工鱼群当前位置
% goal----人工鱼目标位置
% %输出参数：
% H----当前位置人工鱼群的实物浓度

Xf = [];%记录当前鱼群的横坐标
Yf = [];%记录当前鱼群的纵坐标
H = [];%记录当前鱼群的实物浓度
fishnum = size(X,2);

% goal = 381;%目标位置,转化为坐标轴表示方式
[row,col] = ind2sub(n,goal);
[Xg,Yg] = arry2orxy(n,row,col);

%%
%将人工鱼位置转化为坐标值
for i =1:1:fishnum 
    [row,col] = ind2sub(n,X(i));
    [Xf(i),Yf(i)] = arry2orxy(n,row,col);
end
%计算实物浓度的欧氏距离
for i = 1:1:fishnum
    H(i) = sqrt((Xf(i)-Xg)^2+(Yf(i)-Yg)^2);
end

end

