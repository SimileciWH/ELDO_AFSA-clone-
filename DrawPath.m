function DrawPath(n,arrayValue)
%输入参数
% 矩阵维数----n
% 需要画出的矩阵数值-----arrayValue
%输出参数： 无

%13,34,256是栅格表中，从1-n*n的数值
% [row,col] = ind2sub(20,[13 34 256]);%栅格中的数值转化成数组行列值
[row,col] = ind2sub(n,arrayValue);%栅格中的数值转化成数组行列值
[array_x,array_y] = arry2orxy(n,row,col);%将矩阵下标转换为坐标轴xy形式
% scatter(array_x+0.5,array_y+0.5,'MarkerEdgeColor',[0 0 1],'LineWidth',1.5);%在栅格图中表示出来
% plot(array_x+0.5,array_y+0.5,' g-.s  ');%将保留的最优路径点用折线图连接起来
% plot(array_x+0.5,array_y+0.5,' b-h ');%将保留的最优路径点用折线图连接起来
% plot(array_x+0.5,array_y+0.5,'  k--p ');%将保留的最优路径点用折线图连接起来
plot(array_x+0.5,array_y+0.5,' r-o ');%将保留的最优路径点用折线图连接起来
end

%第二种输入方式
%function DrawPath(n,row,col)
%输入参数
%矩阵维数----n
%矩阵的行----row
%矩阵的列----col
%输出参数： 无
% [array_x,array_y] = arry2orxy(n,row,col);%将矩阵下标转换为坐标轴xy形式
% scatter(array_x+0.5,array_y+0.5,'MarkerEdgeColor',[0 0 1],'LineWidth',1.5);%在栅格图中表示出来
% plot(array_x+0.5,array_y+0.5,'-b');%将保留的最优路径点用折线图连接起来
% end

% PS:数值后面加0.5是为了保证，点居于坐标轴栅格中心处
