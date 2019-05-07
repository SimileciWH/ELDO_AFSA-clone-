function [ path_optimum,endpath ] = Gridpath_optimum( n,ii,path,Rf,Barrier )
%输入参数：
% n----矩阵维数
% ii----当前路径的人工鱼编号
% path----当前可行解的矩阵
% Rf----优化检测的视野
% Barrier----障碍矩阵
% 输出参数：
% path_optimum----当前人工鱼优化后的路径矩阵
% endpath----当前检测到哪一个数了
Dt = 0;%原始路径，优化检测视野方位内的距离之和
Dop = 0;%优化后的路径之和
wait_path = [];
wait_test = path(ii);%等待测试矩阵
% endpath = ii;
count = 0;%配合清理未全部修改的格子
search_area = allow_fun(n,Barrier,wait_test,Rf);%优化检测搜寻区域
[cin,ia,ib] = intersect(search_area,path);%找出两个矩阵的公共元素，ia为公共元素在第一个矩阵中的位置，ib类似
%找到ii时刻之后的点在ib中存储的位置,优化它们
ib = sort(ib);%因为intersect之后顺序被打乱了

index = ib((find(ib > ii))');
if isempty(index)
    path_optimum = path;
    endpath = ii;
    return;
end
%在调试过程中发现，当path中有重解时，index会不连续，因此，需要将其补连续了
if index(end) - index(1)+1 ~= length(index)
    index = index(1):1:index(end);
end
%计算ii到最后搜寻视野中的点的欧式距离
for i =1:1:length(index)
    D(i) = distance(n,path(index(i)-1),path(index(i)));
    Dt = Dt+D(i);
end
%搜寻其他路径到达视野中最后位置的路径
for i = 1:1:length(index)
    allow_area = allow_fun(n,Barrier,wait_test,sqrt(2));
    for j = 1:1:length(allow_area)
        DH(j) = distance(n,allow_area(j),path(index(end)));
    end
    [DHmin,index_DH] = min(DH);
    %将距离最小的那个位置放到候选矩阵中去
    wait_path(i) = allow_area(index_DH);
    wait_test = wait_path(i);
    if DHmin == 0   %说明已经找到路径。存放在了wait_path中
        break;
    end
end
% wait_path
%计算候选路径的长度
wait_path_total = [path(ii) wait_path];
for i = 1:1:length(wait_path_total)-1
    dop(i) = distance(n,wait_path_total(i),wait_path_total(i+1));
    Dop = Dop+dop(i);
end

if Dop < Dt%表明需要优化
    disp("need optimum!!!")
    %将不需要的点换成需要的点,只要不是最后一个点，全部覆盖
    for i = 1:1:length(index)
        if path(index(i)) ~= path(index(end))
            path(index(i)) = wait_path(i);   
            if path(index(i)) == path(index(end))
                count = i;
                break;%说明优化完成了
            end
        end
    end
    
    if count ~= 0%说明没有优化完成，非自然结束
        while length(index) - count > 0%说明还有没有修改掉数值的格子
            disp("还有没有修改的格子！！！")
            %判断后面是否还有需要修改但是没有修改的数据
           count = count+1;
            if index(count) ~= index(end)
                disp("改")
                path(index(count)) = path(index(end));
            end 
        end  
    end
end

% if isempty(index)
%     index(end) = length(path);
% end
path_optimum = path;
endpath = index(end);
%--------------------------------------------------更新路径后，路径维数不改变,但是要跳过已经优化过的值---------------------------------
end

% [20,39,58,57,77,96,115,114,113,112,132,131,152,171,170,189,188,207,227,226,245,264,263,262,282,302,322,341,361,381]
% [20,19,38,57,57,77,96,95,114,133,132,131,150,169,168,188,208,227,246,245,226,245,244,263,282,302,322,341,361,381]