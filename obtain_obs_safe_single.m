function [safe_obs,safe_type,obs_id,type_num]=obtain_obs_safe_single(path_safe_flag,param_all)
[rowCount, colCount] = size(param_all);    %%%%%[3,4,3] 
range_all=[length(param_all{1,1})];
for k=1:(colCount-1)                    
    range_all=[range_all,range_all(end)+length(param_all{1,k+1})];%%%%[3,7,10]
end

safe_obs=[];safe_type=[];
for i=1:length(path_safe_flag)
    num=path_safe_flag(i);
    index=find(num > range_all, 1, 'last');
    
    %%%%判断属于哪个部分
    if isempty(index)

        safe_obs=[safe_obs,1];
        safe_type=[safe_type,num];
    else

        safe_obs=[safe_obs,index+1];
        safe_type=[safe_type,num-range_all(index)];

    end
end

% 使用 unique 函数获取唯一值
obs_id = unique(safe_obs);

% 统计每个唯一值的出现次数
type_num = histc(safe_obs, obs_id);

end