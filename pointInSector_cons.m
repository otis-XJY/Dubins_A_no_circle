function obs_range_apf = pointInSector_cons(obs_unknow,obs_range_apf,X_end_A_dubins)
obs=obs_unknow(obs_range_apf,:);
sharp_points = boundary(obs(:,1), obs(:,2));



angle=[obs(:,2)-X_end_A_dubins(2),obs(:,1)-X_end_A_dubins(1)];
if ~isempty(angle)
    distance=sqrt(angle(:,1).^2+angle(:,2).^2);
    
    
    [unique_values, ~, idx] = unique(angle);
    
    % 找到出现次数超过一次的值
    duplicates = unique_values(histcounts(idx, 1:numel(unique_values)+1) > 1);
    
    % 找到重复值在原始数组中的索引
    indices = arrayfun(@(x) find(angle == x), duplicates, 'UniformOutput', false);
    
    for i=1:length(indices)
        if find(find(diff(indices{i})~=1))
            [max_value, index] = min(distance(indices{i}));
            id_all=indices{i};
            keep_id=indices{i}(index);
            id_all(keep_id)=[];
            obs_range_apf(id_all)=[];
        end
    end

end




