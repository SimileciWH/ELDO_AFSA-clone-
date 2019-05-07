function [ allow_area ] = allow_fun( n,Barrier,Xi,Rf )
%计算当前人工鱼的可行域
%输入参数：
% n----矩阵维数
% Barrier----障碍物
% Xi----当前人工鱼的位置
% rightInf-Rf----可行域上阶，可以理解为视野
% 输出参数：
% allow_area----当前人工鱼的可行域
    j = 1;%记录单个鱼可行域的个数
    A = 1:1:n^2;
    allow = setdiff(A,Barrier);    
    for i = 1:1:length(allow)
        if 0 <= distance(n,Xi,allow(i)) && distance(n,Xi,allow(i)) <= Rf  
            allow_area(j) = allow(i);
            j = j+1;
        end
    end

end
