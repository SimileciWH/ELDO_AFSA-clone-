function  [array_x,array_y] = arry2orxy( n,row,col )
%把矩阵下标转换成坐标轴坐标
%输入矩阵的
% 维数----n
% 行----row
% 列----col
%输出坐标轴
% 横坐标----arrayx
% 纵坐标----arrayy
k = length(row);
for i = 1:1:k
    array_x(i) = col(i);
    array_y(i) = row(i);
end

end

