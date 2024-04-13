function [flag_safe,obs_id]=if_safe(vtheta,path,obs,from_path,to_path,point,obs_know)
Distance={};
% start_=[Start_Point(1:2),10];
% end_=[Start_Point(1:2),10];
% obs_all=[obs;start_;end_];

if to_path==2
    start_sqre=point(:,3);
else
    start_sqre=point(:,4);
end

end_sqre=point(:,1);


R_MAX=min(obs(:,3));
% if pos_id==-1 &&parent_id==-1
%     start_sqre=Start_Point;
%     end_sqre=Start_Point;
% elseif pos_id==-2 && parent_id==-1
%     start_sqre=End_Point;
%     end_sqre=Start_Point;
% elseif parent_id==-1
%     start_sqre=start_point;
%     end_sqre=Start_Point;
% elseif pos_id==-2
%     start_sqre=End_Point;
%     end_sqre=end_point;
% else
%     start_sqre=start_point;
%     end_sqre=end_point;
% end


angle2=atan2(start_sqre(2)-end_sqre(2),start_sqre(1)-end_sqre(1));
if angle2+pi/2>pi
    angle1=angle2+pi/2-pi;
else
    angle1=angle2+pi/2;
end

x=[start_sqre(1)+cos(angle2)*R_MAX+cos(angle1)*R_MAX,start_sqre(1)+cos(angle2)*R_MAX-cos(angle1)*R_MAX,end_sqre(1)-cos(angle2)*R_MAX-cos(angle1)*R_MAX,end_sqre(1)-cos(angle2)*R_MAX+cos(angle1)*R_MAX];
y=[start_sqre(2)+sin(angle2)*R_MAX+sin(angle1)*R_MAX,start_sqre(2)+sin(angle2)*R_MAX-sin(angle1)*R_MAX,end_sqre(2)-sin(angle2)*R_MAX-sin(angle1)*R_MAX,end_sqre(2)-sin(angle2)*R_MAX+sin(angle1)*R_MAX];
x=[x,x(1)];
y=[y,y(1)];

x_point=obs_know(:,1);
y_point=obs_know(:,2);


% plot(x, y, 'b-', 'LineWidth', 2);hold on
% hold off
x_min=min(x);
x_max=max(x);
y_min=min(y);
y_max=max(y);

in = x_point >= x_min & x_point <= x_max & y_point >= y_min & y_point <= y_max;




% in = inpolygon(x_point,y_point, x, y);

id=find(in==1);

obs_id=obs_know(id,3);

[obs_to_avoid, ~, ~] = unique(obs_id);

obs=obs(obs_to_avoid,:);

% [Y,I] = sort(obs_all(:,1)); %对OpenList中第三列排序
% obs_all_x=obs_all(I,:);%open中第一行节点是F值最小的  重新排序open为从大到小
% start_id_x=find(obs_all_x(:,1)==obs(pos_id,1));
% end_id_x=find(obs_all_x(:,1)==obs(father_id,1));
% 
% 
% [Y,I] = sort(obs_all(:,2)); %对OpenList中第三列排序
% obs_all_y=obs_all(I,:);%open中第一行节点是F值最小的  重新排序open为从大到小
% 
% 
% 
% 
% if father_id>pos_id
%     obs_x=obs_all_x(pos_id:father_id,:);
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 

obs_id=[];







Distance={};
  %% （4种）整条路径距离障碍圆心的最小值
  for k=1:length(obs(:,1))
    for j=from_path:to_path
        for i = 1:length(path{j}(1,:))
            Distance{j}(i) = sqrt((path{j}(1,i)-obs(k,1))^2+(path{j}(2,i)-obs(k,2))^2);
%             if Distance{j}(i)>= obs(k,3)
%                 break;
%             end
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
        obs_id=[obs_id;obs_to_avoid(k)];
    end
  end


if ~exist('flag_safe', 'var')
    flag_safe=1;
end








%   %% （4种）整条路径距离障碍圆心的最小值
%   for k=1:length(obs(:,1))
%   flag_safe(k)=1;
%     for j=from_path:to_path
%         if j==1||j==3
%             if isempty(path{j})
%                 flag_safe(k)=0;
%                 obs_id=[obs_id;id(k)];
%             end
%             for i = 1:length(path{j}(1,:))
%                 Distance{j}(i) = sqrt((path{j}(1,i)-obs(k,1))^2+(path{j}(2,i)-obs(k,2))^2);
%     %             display(Distance{j}(i))
%                 if Distance{j}(i)<obs(k,3)
%                     flag_safe(k)=0;
%                     obs_id=[obs_id;id(k)];
%                     break;
%                 end
%             end
% %     %         display(Distance)
% %     %         display(j)
% %             if ~isempty(path{j})
% %                 mDistance_point(j) = min(Distance{j});
% %             else
% %                 mDistance_point(j)=0;
% %             end
%         elseif j==2
% %             display(path{j}(1,1))
%             if isempty(path{j})
%                 flag_safe(k)=0;
%                 obs_id=[obs_id;id(k)];
%             else
% 
%             x0=path{j}(1,1);
%             y0=path{j}(2,1);%%%%直线上一点
%             m=tan(vtheta);%%%%斜率
%             
%             a=m;
%             b=-1;
%             c=y0-m*x0;
%             distances = abs((a * obs(k,1) + b * obs(k,2) + c) / sqrt(a^2 + b^2));
%             if distances<obs(k,3)
%                 flag_safe(k)=0;
%                 obs_id=[obs_id;id(k)];
%                 break;
%             end
%             end
%         end
% 
% % 
% 
% 
% 
% 
% 
% 
%     end
% %     display(Distance)
% %     display(mDistance_point)
%     min_dis=min(mDistance_point);
% %     display([min_dis,fl])
%     error = 0.0001;
%     %% 选择最优
%     if min_dis >= obs(k,3) - error%最短的是否不过障碍圆
%         flag_safe(k)=1;
%     else
%         flag_safe(k)=0;
%     end
% 
% 
%   end


end