function [ D ] = eachAF_dist(n,N,ppValue,position )
%输入参数：
% n----矩阵维数
% N----人工鱼条数
% ppValue----当前人工鱼的位置
% position----所有人工鱼的位置
% 输出参数：
% D----当前人工鱼与其他人工鱼的位置，包括自身在内.

D = zeros(1,N);
%将当前人工鱼位置转换成坐标
[pprow,ppcol] = ind2sub(n,ppValue);%栅格中的数值转化成数组行列值
[pp_array_x,pp_array_y] = arry2orxy(n,pprow,ppcol);%将矩阵下标转换为坐标轴xy形式

for i = 1:1:N
 
    [positionrow,positioncol] = ind2sub(n,position(i));%栅格中的数值转化成数组行列值
    [position_array_x,position_array_y] = arry2orxy(n,positionrow,positioncol);%将矩阵下标转换为坐标轴xy形式
    
    D(i) = norm([pp_array_x,pp_array_y] - [position_array_x,position_array_y]);
end

end

