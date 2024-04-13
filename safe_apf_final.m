function safe_final=safe_apf_final(X_now_APF,X_APF_final,obs_unknow,obs_range,obs_no_circle,r_threta)

% X_APF_final

safe_final=false;

path_x_positive=[];
path_x_negative=[];
path_y_positive=[];
path_y_negative=[];
singal_x=[];
singal_y=[];
%     plot(path(1,now_id+i-1),path(2,now_id+i-1),'x', 'Color', 'y','LineWidth',5)
for j=1:length(obs_range)
    singal_x=[singal_x,obs_unknow(obs_range(j),1)-X_APF_final(1)];
    singal_y=[singal_y,obs_unknow(obs_range(j),2)-X_APF_final(2)];
end

dis=sqrt((X_now_APF(1)-X_APF_final(1))^2+(X_now_APF(1)-X_APF_final(1))^2);



path_x_positive=[path_x_positive,sum(singal_x>0)];
path_x_negative=[path_x_negative,sum(singal_x<0)];
path_y_positive=[path_y_positive,sum(singal_y>0)];
path_y_negative=[path_y_negative,sum(singal_y<0)];


safe_flag=[];
if all(path_x_positive == 0)||all(path_x_negative == 0)||all(path_y_positive == 0)||all(path_y_negative == 0) || dis<r_threta
    
    for k=1:length(obs_no_circle)
        if ~isempty(obs_no_circle{k})
            x=obs_no_circle{k}(:,1);
            y=obs_no_circle{k}(:,2);
            in = inpolygon(X_APF_final(1),X_APF_final(2), x, y);
    
            if all(in~=1)
                safe_flag=[safe_flag,true];
            else
                safe_flag=[safe_flag,false];
            end
        end
    end
    


    if all(safe_flag==1)
        safe_final=true;
    end
end

end



