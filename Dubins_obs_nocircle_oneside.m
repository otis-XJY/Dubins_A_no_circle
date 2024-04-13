function [param_safe,param_best] = Dubins_obs_nocircle_oneside(Start_Point,End_Point,center1,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle)

param_all={};param_best={};
% display('start find 1-5')
[param_all{1},param_best{1}]=Dubins_obs(Start_Point,End_Point,center1,r_s,r_e,R,Stepsize,pos_id,parent_id);
[flag_all,obs_id_all]=obtain_flag_safe(5,param_all,obs_no_circle,outline);
[min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(2,param_all,flag_all);



% display('end find 1-5')
if ~isempty(path_safe_flag)
    display('directly find')
    param_safe=struct();
    param_safe=obtain_safeparam(path_safe_flag,param_all);
    param_safe=param_safe{1};
else
%     display('start find 1-2')
    [flag_all_1,obs_id_all_1]=obtain_flag_safe(2,param_all,obs_no_circle,outline);%%%检查前两段
    [min_path_1,path_safe_flag_1,flag_min_1]=obtain_min_or_onlypath_from_safe(1,param_all,flag_all_1);
%     display('start find 3-5')
    [flag_all_2,obs_id_all_2]=obtain_flag_safe(3,param_all,obs_no_circle,outline);%%检查3-5段
    [min_path_2,path_safe_flag_2,flag_min_2]=obtain_min_or_onlypath_from_safe(1,param_all,flag_all_2);
    %%
    if flag_min_1==0 && flag_min_2~=0%%%%前两段不安全
        
        param_safe=struct();
        param_safe=obtain_safeparam(path_safe_flag_2,param_all);
        param_safe=param_safe{1};
        End_Point_now=[param_all{1}(path_safe_flag_2(flag_min_2)).point(:,3)',param_all{1}(path_safe_flag_2(flag_min_2)).vtheta(2)];
        if if_safe_point(obs_no_circle,End_Point_now,r_e)
            display('try find in first')
        [param_safe,param_best] = Dubins_obs_nocircle_oneside_first(Start_Point,End_Point,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle,End_Point_now,param_safe); 
        end
    end
    %%
    if flag_min_1~=0 && flag_min_2==0%%%%3-5段不安全
        
        param_safe=struct();
        param_safe=obtain_safeparam(path_safe_flag_1,param_all);
        param_safe=param_safe{1};
%         flag_min_1
%         path_safe_flag_1
%         param_all{1}
%         param_all{1}(())

        Start_point_now=[param_all{1}(path_safe_flag_1(flag_min_1)).point(:,3)',param_all{1}(path_safe_flag_1(flag_min_1)).vtheta(2)];
        if if_safe_point(obs_no_circle,Start_point_now,r_e)
            display('try find in end')
        [param_safe,param_best] = Dubins_obs_nocircle_oneside_end(Start_Point,End_Point,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle,Start_point_now,param_safe);  
        end
    end
%     if flag_min_1==0 && flag_min_2==0 %%%%%都不安全
% %         outline_all_positive
%         display('aaaaaaaaaaaa')
%         m=(End_Point(2)-Start_Point(2))/(End_Point(1)-Start_Point(1));%%%%斜率
%         m=-1/m;
%         a=m;
%         b=-1;
%         c=center1(2)-m*center1(1);
%         points=outline_all_positive(:,1:2);
%         distances = (a*points(:,1) + b*points(:,2) + c) / sqrt(a^2 + b^2);
% 
%         % 找到距离最大的点及其索引
%         [max_distance, max_index] = max(distances);
%         [min_distance, min_index] = min(distances);
% 
% 
%         outline_all_positive_new=outline_all_positive(find(distances>=0),:);
%         outline_all_negative_new=outline_all_positive(find(distances<0),:);
%         
%         
%         % 输出距离最大的点
%         farthest_point_max = points(max_index, :);
%         farthest_point_min = points(min_index, :);
% 
% 
% 
% 
%         if ~isempty(outline_all_positive_new) && ~isempty(outline_all_negative_new)
%         %     display(['avoid  ',num2str(pos_id),'  up'])
%          if if_safe_point(obs_no_circle,farthest_point_max)
%             [param_safe1,param_best1] = Dubins_obs_nocircle_oneside(Start_Point,End_Point,farthest_point_max,outline_all_positive_new,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
%          end
%             %     display(['avoid  ',num2str(pos_id),'  up   end'])
%         %     display(['avoid  ',num2str(pos_id),'  down'])
%          if if_safe_point(obs_no_circle,farthest_point_min)
%             [param_safe2,param_best2] = Dubins_obs_nocircle_oneside(Start_Point,End_Point,farthest_point_min,outline_all_negative_new,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
%         %     display(['avoid  ',num2str(pos_id),'  down  end'])
%          end
%         end
% 
% 
%     end


end

if ~exist('param_safe', 'var')
    param_safe=struct();
    
end





