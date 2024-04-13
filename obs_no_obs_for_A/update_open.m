function [open_f,open]=update_open(path_safe_flag,param_all,End_Point,open_f,open)
[rowCount, colCount] = size(param_all);    %%%%%[3,4,3] 
range_all=[length(param_all{1,1})];
for k=1:(colCount-1)                    
    range_all=[range_all,range_all(end)+length(param_all{1,k+1})];%%%%[3,7,10]
end


for i=1:length(path_safe_flag)
        num=path_safe_flag(i);
    index=find(num > range_all, 1, 'last');
    
    %%%%判断属于哪个部分
    if isempty(index)
        param=param_all{1,1}(num);
    else
        param=param_all{1,index+1}(num-range_all(index));

    end
%     param=param_all{ceil(path_safe_flag(i)/4)}(path_safe_flag(i)-4*(ceil(path_safe_flag(i)/4)-1));
    %%索引路径参数第几（初始半径）行第几（躲避障碍圆）个，具体第几个路径
    node.point=param.point(:,2:3);
    node.vtheta=param.vtheta(2);
    node.g=sum(param.length(1:2));
    node.f=node.g+obtain_h(node.point(:,2)',End_Point);
    node.path{1}=param.path{1};
    node.path{2}=param.path{2};
    node.R=param.R;
    node.r=param.r;%%[r_s,r_e]
    node.center=param.center;
    open_f=[open_f,node.f];
    open{end+1}=node;
end
end