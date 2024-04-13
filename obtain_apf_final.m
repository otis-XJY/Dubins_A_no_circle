function [X_APF_final,X_APF_final_id]=obtain_apf_final(now_id,l,draw_path,obs_unknow,obs_range_apf,obs_no_circle,type)
%%%%l为搜索步长
%%%%%%%%now_id=相对于draw_path而言，注意换算
%%%%draw_path最终所有的路径点

X_APF_final=[];
if type==1%%防止直接推到终点
for i=0:l:length(draw_path)-2000
    if now_id+i>length(draw_path)-2000
        i=length(draw_path)-now_id-2001;
    end
%     plot(draw_path(1,now_id+i),draw_path(2,now_id+i),'v')
%     pause(0.01)
    path_safe1=point_in_path(obs_unknow,obs_range_apf,draw_path,1,now_id+i);%%%%判断这个点对于所有的观测障碍物而言是否安全
    if path_safe1
        path_safe2=safe_apf_final(draw_path(:,now_id+i),draw_path(:,now_id+i),obs_unknow,[],obs_no_circle,nan);
        if path_safe2
            X_APF_final=draw_path(:,now_id+i);
            X_APF_final_id=now_id+i;
            break
        end

    end
end
else
    for i=0:l:length(draw_path)
    if now_id+i>length(draw_path)
        i=length(draw_path)-now_id-1;
    end
%     plot(draw_path(1,now_id+i),draw_path(2,now_id+i),'v')
%     pause(0.01)
    path_safe1=point_in_path(obs_unknow,obs_range_apf,draw_path,1,now_id+i);%%%%判断这个点对于所有的观测障碍物而言是否安全
    if path_safe1
        path_safe2=safe_apf_final(draw_path(:,now_id+i),draw_path(:,now_id+i),obs_unknow,[],obs_no_circle,nan);
        if path_safe2
            X_APF_final=draw_path(:,now_id+i);
            X_APF_final_id=now_id+i;
            break
        end

    end
    end
end

if isempty(X_APF_final)
    X_APF_final=draw_path(:,now_id+i);
    X_APF_final_id=now_id+i;
end


