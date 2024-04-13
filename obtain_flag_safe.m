function [flag_all,obs_id_all]=obtain_flag_safe(type,param_all,obs_no_circle,outline_all)%type,param_all,obs,
switch type
    case 1  %%%检查所有(1-3)
        [flag_all,obs_id_all] = obtian_all_or_part(param_all,obs_no_circle,outline_all,1,3);%param_all,obs
    case 2  %%%检查前两段
        [flag_all,obs_id_all] = obtian_all_or_part(param_all,obs_no_circle,outline_all,1,2);
    case 3  %%%检查3-5段
        [flag_all,obs_id_all] = obtian_all_or_part(param_all,obs_no_circle,outline_all,3,5);
    case 5  %%%检查5段
        [flag_all,obs_id_all] = obtian_all_or_part(param_all,obs_no_circle,outline_all,1,5);
    otherwise
        warning('No type')
end



function [flag_all,obs_id_all]=obtian_all_or_part(param_all,obs_no_circle,outline_all,from_path,to_path)
[rowCount, colCount] = size(param_all);
flag_all={};
obs_id_all=[];

if rowCount==1
    for j=1:colCount%%针对每个障碍物
        for i=1:length(param_all{j})
            if ~isempty(param_all{j}(i).path) && ~isnan(param_all{j}(i).Length)
                if ~isempty(outline_all)
                [flag_safe,obs_id]=if_safe_no_circle(param_all{j}(i).vtheta(2),param_all{j}(i).path,obs_no_circle,outline_all,from_path,to_path,param_all{j}(i).point);
                flag_all{end+1}=flag_safe;
                obs_id_all=[obs_id_all;obs_id];
                else
                    flag_all{end+1}=1;
                end
            else
                flag_all{end+1}=0;
            end
        end
    end
else
    flag_all_r={};
    flag_all_R={};
    for j=1:colCount%%针对每个障碍物
        for i=1:length(param_all{1,j})%%针对每个路段
            if ~isempty(param_all{1,j}(i).path) && ~isnan(param_all{1,j}(i).Length)
                [flag_safe_r,obs_id]=if_safe_no_circle(param_all{1,j}(i).vtheta(2),param_all{1,j}(i).path,obs_no_circle,outline_all,from_path,to_path,param_all{1,j}(i).point);
                flag_all_r{end+1}=flag_safe_r;
                obs_id_all=[obs_id_all;obs_id];
            else
                flag_all_r{end+1}=0;
            end
            
        end
        for i=1:length(param_all{2,j})%%针对每个路段
            if ~isempty(param_all{2,j}(i).path) && ~isnan(param_all{2,j}(i).Length)
                [flag_safe_R,obs_id]=if_safe_no_circle(param_all{2,j}(i).vtheta(2),param_all{2,j}(i).path,obs_no_circle,outline_all,from_path,to_path,param_all{2,j}(i).point);
                flag_all_R{end+1}=flag_safe_R;
                obs_id_all=[obs_id_all;obs_id];
            else
                flag_all_R{end+1}=0;
            end
        end
    end
    flag_all.R=flag_all_R;
    flag_all.r=flag_all_r;
end