function flag_safe=if_safe_old(path,obs,from_path,to_path)
Distance={};
  %% （4种）整条路径距离障碍圆心的最小值
  for k=1:length(obs(:,1))
    for j=from_path:to_path
        for i = 1:length(path{j}(1,:))
            Distance{j}(i) = sqrt((path{j}(1,i)-obs(k,1))^2+(path{j}(2,i)-obs(k,2))^2);
%             display(Distance{j}(i))
        end
%         display(Distance)
%         display(j)
        if ~isempty(path{j})
            mDistance_point(j) = min(Distance{j});
        else
            mDistance_point(j)=0;
        end
    end
%     display(Distance)
%     display(mDistance_point)
    min_dis=min(mDistance_point);
%     display([min_dis,fl])
    error = 0.0001;
    %% 选择最优
    if min_dis >= obs(k,3) - error%最短的是否不过障碍圆
        flag_safe(k)=1;
    else
        flag_safe(k)=0;
    end
  end


end