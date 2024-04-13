function [param_safe,param_best] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,center1,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle)

param_all={};param_best={};
% display('start find 1-5')
[param_all{1},param_best{1}]=Dubins_obs(Start_Point,End_Point,center1,r_s,r_e,R,Stepsize,pos_id,parent_id);
[flag_all,obs_id_all]=obtain_flag_safe(2,param_all,obs_no_circle,outline);%%%检查1-2段
[min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(2,param_all,flag_all);



% display('end find 1-5')
if ~isempty(path_safe_flag)
    display('directly find 1-2 safe')
    param_safe=struct();
    param_safe=obtain_safeparam(path_safe_flag,param_all);
    param_safe=param_safe{1};
else
%         outline_all_positive
    
    flag_all_point=0;
    for i=1:length(param_all{1})
        if ~isempty(param_all{1}(i).point)
            flag=if_safe_point(obs_no_circle,param_all{1}(i).point(:,3)',0);
            if flag==1
                flag_all_point=i;
                break
            end
        end
    end
    if flag~=0
        display('需要插入点')

        End_Point_now=[param_all{1}(flag_all_point).point(:,3)',param_all{1}(flag_all_point).vtheta(2)];
    
    
        [param_safe,param_best] = Dubins_obs_nocircle_oneside_first(Start_Point,End_Point,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle,End_Point_now,param_all{1});
        
    else
        display('没有安全的初始=>不查找路径')
    end


end



if ~exist('param_safe', 'var')
    param_safe=struct();
    
end





