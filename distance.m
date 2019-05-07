function [ dist ] = distance(n,ppValue,randomValue )
%输入参数：
%n----矩阵的维数
% ppValue----当前位置
% randomValue----任意位置
% 输出参数：
% dist----两个元素之间的距离

[pprow,ppcol] = ind2sub(n,ppValue);%栅格中的数值转化成数组行列值
[pp_array_x,pp_array_y] = arry2orxy(n,pprow,ppcol);%将矩阵下标转换为坐标轴xy形式

[randrow,randcol] = ind2sub(n,randomValue);%栅格中的数值转化成数组行列值
[rand_array_x,rand_array_y] = arry2orxy(n,randrow,randcol);%将矩阵下标转换为坐标轴xy形式

dist = sqrt((pp_array_x-rand_array_x)^2+(pp_array_y-rand_array_y)^2);

end

