function [ nextPosition,nextPositionH ] = GridAF_follow( n,N,ppValue,ii,visual,delta,try_number,lastH,Barrier,goal)
%输入参数：
% n----矩阵维数
% N----人工鱼总数
% ppValue----所有人工鱼群的位置
% ii----当前人工鱼群的编号
% visual----感知范围
% delta----拥挤因子
% try_number----尝试次数
% LastH----当前人工鱼上一次的食物浓度
% Barrier----障碍物矩阵
% goal----人工鱼目标位置
% 输出参数：
% nextPosition----下一时刻该条人工鱼的位置
% nextPositionH----下一时刻该条人工鱼位置处的食物浓度

Xi = ppValue(ii);%当前人工鱼的位置
D = eachAF_dist(n,N,Xi,ppValue);%计算当前人工鱼与其他所有与群的欧式距离
index = find(D > 0 & D < visual);%找到视野中的其他鱼群
Nf = length(index);%确定视野之内的鱼群总数
j = 1;%记录单个鱼可行域的个数
rightInf = sqrt(2);
%%
%计算当前人工鱼的可行域
A = 1:1:n^2;
allow = setdiff(A,Barrier);    
for i = 1:1:length(allow)
    if 0 < distance(n,Xi,allow(i)) && distance(n,Xi,allow(i)) <= rightInf  
        allow_area(j) = allow(i);
        j = j+1;
    end
end
%%
if Nf > 0          %Nf > 0说明视野之内有其他人工鱼群，则可以进行群聚行为
%----------------------------------------计算出Hmin，对应的Xmin--------------------------------------------------------------------
    Xvisual = ppValue(index);%取出与当前人工鱼邻近的鱼群的坐标
    Hvisual = lastH(index);%取出与当前人工鱼邻近的鱼群的食物浓度
    [Hmin,minindex] = min(Hvisual);%求出邻近鱼群中食物浓度最低的值
    Xmin = Xvisual(minindex);%得到食物浓度最低的鱼群位置
%-------------------------------------------------------------------------------------------------------------    
    Hi = lastH(ii);%当前人工鱼的实物浓度
    if Hmin/Nf <= Hi*delta     %如果中心位置的食物浓度比当前位置高，并且不拥挤，则向中心位置走一步      PS：H值越小，则离目标越近
        %在可行域中，随机走一步，比较到中心点距离与当前值到中心点距离，谁小就取谁
        for i = 1:1:try_number
            Xnext = allow_area(uint16(rand*(length(allow_area)-1)+1));%在可行域中随机选择一步.
            if distance(n,Xnext,Xmin) < distance(n,Xi,Xmin)
                nextPosition = Xnext;
                nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);
                break;
            else
                nextPosition = Xi;
                nextPositionH = GrideAF_foodconsistence(n,nextPosition,goal);
            end
        end
    else     %如果中心位置的食物浓度没有当前位置的食物浓度高，则进行觅食行为
        [nextPosition,nextPositionH] = GridAF_prey(n,ppValue(ii),ii,try_number,lastH,Barrier,goal);
    end
    
else                %否则，Nf < 0说明视野范围内没有其他人工鱼，那么就执行觅食行为  
    [nextPosition,nextPositionH] = GridAF_prey(n,ppValue(ii),ii,try_number,lastH,Barrier,goal);
end




end

