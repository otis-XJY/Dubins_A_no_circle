function [flag_safe,obs_id]=if_safe_no_circle(vtheta,path,obs_no_circle,outline,from_path,to_path,point)

Distance={};

num=length(path);

if from_path==1
    from_path=1;
else
    from_path=num-2;
end

if to_path==2
    to_path=num-3;
    start_sqre=point(:,num+1-3);
elseif to_path==3
    to_path=3;
    start_sqre=point(:,4);
else
    to_path=num;
    start_sqre=point(:,num+1);
end













% if to_path==2
%     start_sqre=point(:,3);
% else
%     start_sqre=point(:,4);
% end

end_sqre=point(:,1);
R_MAX=10;



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

x_point=outline(:,1);
y_point=outline(:,2);


% plot(x, y, 'b-', 'LineWidth', 2);hold on
% hold off
x_min=min(x);
x_max=max(x);
y_min=min(y);
y_max=max(y);

in = x_point >= x_min & x_point <= x_max & y_point >= y_min & y_point <= y_max;




% in = inpolygon(x_point,y_point, x, y);

id=find(in==1);

obs_id=outline(id,3);

[obs_to_avoid, ~, ~] = unique(obs_id);

obs_no_circle=obs_no_circle(obs_to_avoid);


obs_id=[];


Distance={};
% from_path
% to_path
  %% （4种）整条路径距离障碍圆心的最小值
  for k=1:length(obs_no_circle)
      flag_safe(k)=1;
      x=obs_no_circle{k}(:,1);
      y=obs_no_circle{k}(:,2);
    for j=from_path:to_path
        for i = 1:length(path{j}(1,:))
            in = inpolygon(path{j}(1,i),path{j}(2,i), x, y);
            if in
                flag_safe(k)=0;
                obs_id=[obs_id;obs_to_avoid(k)];
                break;
            end

        end

    end
  end


if ~exist('flag_safe', 'var')
    flag_safe=1;
end

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


