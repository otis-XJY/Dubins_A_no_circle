function flag_all=obtain_flag_safe(type,param_all,obs)%type,param_all,obs,
switch type
    case 1%%%检查所有
        flag_all = obtian_all_or_part(param_all,obs,3);%param_all,obs
    case 2%%%检查前两段
        flag_all = obtian_all_or_part(param_all,obs,2);
    otherwise
        warning('No type')
end



function flag_all=obtian_all_or_part(param_all,obs,type)
[rowCount, colCount] = size(param_all);
if rowCount==1
    for j=1:colCount%%针对每个障碍物
        for i=1:length(param_all{j})
            if ~isempty(param_all{j}(i).path)
                flag_safe=if_safe(param_all{j}(i).path,obs,1,type);
                flag_all((i+4*(j-1)),:)=flag_safe;
            else
                flag_all(end+1,:)=zeros(1,length(obs));
            end
        end
    end
else
    flag_all_r=[];
    flag_all_R=[];
    for j=1:colCount%%针对每个障碍物
        for i=1:length(param_all{1,j})%%针对每个路段
            if ~isempty(param_all{1,j}(i).path)
                flag_safe_r=if_safe(param_all{1,j}(i).path,obs,1,type);
                flag_all_r(end+1,:)=flag_safe_r;
            else
                flag_all_r(end+1,:)=zeros(1,length(obs));
            end
            
        end
        for i=1:length(param_all{2,j})%%针对每个路段
            if ~isempty(param_all{2,j}(i).path)
                flag_safe_R=if_safe(param_all{2,j}(i).path,obs,1,type);
                flag_all_R(end+1,:)=flag_safe_R;
            else
                flag_all_r(end+1,:)=zeros(1,length(obs));
            end
        end
    end
    flag_all.R=flag_all_R;
    flag_all.r=flag_all_r;
end