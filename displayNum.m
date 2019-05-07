function displayNum(n)
%输入参数
%矩阵维数----n
%无输出
%%
x_text = 1:1:n*n;%产生1-400的数值.
test_num = 32;
%%
%将数值在栅格图上显示出来
for i = 1:1:n*n
    [row,col] = ind2sub(n,i);
    [array_x,array_y] = arry2orxy(n,row,col);
    text(array_x+0.2,array_y+0.5,num2str(x_text(i)));
end
%验证栅格数值与行列值是否对应
[row,col] = ind2sub(n,test_num);
[array_x,array_y] = arry2orxy(n,row,col);
fprintf('the value %d is on array_x = %d,array_y = %d\n',test_num,array_x,array_y);%显示校对信息，供人工检验.
end

