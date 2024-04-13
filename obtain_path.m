function final_path=obtain_path(close)

choose=close;

%%%%%%先确定每个数的父节点
for i=1:length(choose)
    for j=i+1:length(choose)
        if choose{i}.point(:,end)==choose{j}.point(:,1)
            choose{j}.father=choose{i};
        end
    end
end


check_cell=choose{end};
final_path={choose{end}};
while isfield(check_cell, 'father')
    final_path(2:end+1)=final_path(1:end);
    final_path{1}=check_cell.father;
    check_cell=check_cell.father;
end

% Draw_map(Start_Point,End_Point,obs,sure);%绘制地图
% % close=update_close(path_safe_flag(flag_min),param_all,End_Point,close);
% Draw_pathA(final_path,sure);
