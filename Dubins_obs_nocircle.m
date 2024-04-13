function [param_all,param_best] = Dubins_obs_nocircle(Start_Point,End_Point,outline_all,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle,total_field)

display(['from           ',num2str(parent_id), '                  avoid  ',num2str(pos_id)])
% Start_Point = [param_all{1}(1).point(:,2)',param_all{1}(1).vtheta(2)];
% End_Point = [param_all{1}(1).point(:,3)',param_all{1}(1).vtheta(2)];%[x,y,角度]
% r=7;%最小转弯半径
% Stepsize=0.01;%路径的生成间隔

% sure=3;%保证安全的预留距离
% id=1

outline=outline_all(find(outline_all(:,3)==pos_id),:);

points=outline(:,1:2);
% x0=path{j}(1,1);
% y0=path{j}(2,1);%%%%直线上一点
% m=tan(vtheta);%%%%斜率

% plot(points(:,1),points(:,2),'.');hold on

x0=Start_Point(1);
y0=Start_Point(2);%%%%直线上一点
m=(End_Point(2)-Start_Point(2))/(End_Point(1)-Start_Point(1));%%%%斜率

a=m;
b=-1;
c=y0-m*x0;

% 生成直线上的点
x = linspace(100, 200, 100);  % 生成 x 坐标的范围
y = (-a*x - c) / b;          % 根据直线方程计算对应的 y 坐标

% 绘制直线
% plot(x, y, 'b-', 'LineWidth', 2);

% 设置坐标轴范围
% axis equal;

if abs(a)~=inf
% 计算每个点到直线的距离
distances = (a*points(:,1) + b*points(:,2) + c) / sqrt(a^2 + b^2);
else
distances=points(:,1)-Start_Point(1);
end
% 找到距离最大的点及其索引
[max_distance, max_index] = max(distances);
[min_distance, min_index] = min(distances);


outline_all_positive=outline(find(distances>=0),:);
outline_all_negative=outline(find(distances<0),:);


% 输出距离最大的点
farthest_point_max = points(max_index, :);
farthest_point_min = points(min_index, :);


flag_safe_max=if_safe_point(obs_no_circle,farthest_point_max,0);
flag_safe_min=if_safe_point(obs_no_circle,farthest_point_min,0);

% plot(farthest_point1(1),farthest_point1(2),'x')
% plot(farthest_point2(1),farthest_point2(2),'x');hold on
% plot(Start_Point(1),Start_Point(2),'o')
% plot(End_Point(1),End_Point(2),'o')
% 
% r_s=r;
% r_e=sure;

if ~isempty(outline_all_positive) && ~isempty(outline_all_negative)
    display('1')
    if flag_safe_max==1
        display(['1.1'])
    %     outline_all_positive
    %     outline_all_negative
        [param_safe1,param_best1] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,farthest_point_max,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    %     display(['avoid  ',num2str(pos_id),'  up   end'])
    %     display(['avoid  ',num2str(pos_id),'  down'])
    end
    
    if flag_safe_min==1
        display(['1.2'])
        [param_safe2,param_best2] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,farthest_point_min,outline_all_negative,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    %     display(['avoid  ',num2str(pos_id),'  down  end'])
    end
end



if isempty(outline_all_positive)
    display(['2'])
%     center = [mean(obs_no_circle{pos_id}(1:end-1,1)), mean(obs_no_circle{pos_id}(1:end-1,2))];
%     [total_field,boundary]=obtian_partfield(2*60,End_Point,outline_all,1.5,30,center);
    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,0.8*R,farthest_point_max,1,total_field,1);
    flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
    pose=1;
    while flag_safe==0
        pose=pose+1;
        if pose>poit_num
            break
        end
        center_new=obtain_new_center(a,b,c,Start_Point,0.8*R,farthest_point_max,1,total_field,pose);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);


    end
    if flag_safe==0
        display('type 2')
        center_new=obtain_new_center_new(a,b,c,Start_Point,R+1,farthest_point_max,1);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
        k=0;
        while flag_safe==0
            k=k+0.1;
            center_new=obtain_new_center_new(a,b,c,Start_Point,R+1-k,farthest_point_max,1);
            flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
            if k==R+1
                break
            end
        end
    end

    if flag_safe==1
        display(['2 succed 1'])
%         display(center_new)
    [param_safe1,param_best1] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,center_new,outline_all_negative,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    % 计算每个点到直线的距离
    end
%%%%%%%%%%%%%%%%%
distance_all_negative=distances(find(distances<0));
points_negative=points(find(distances<0),:);
[min_distance, min_index] = min(distance_all_negative);

farthest_point_min = points_negative(min_index, :);

    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_min,2,total_field,1);
    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_min,2,total_field,poit_num);
    flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
    pose=poit_num;
    while flag_safe==0
        pose=pose-1;
        if pose<1
            break
        end
        center_new=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_min,2,total_field,pose);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);


    end
    if flag_safe==0
        display('type 2')
        center_new=obtain_new_center_new(a,b,c,Start_Point,R+1,farthest_point_min,2);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
        k=0;
        while flag_safe==0
            k=k+0.1;
            center_new=obtain_new_center_new(a,b,c,Start_Point,R+1-k,farthest_point_min,2);
            flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
            if k==R+1
                break
            end
        end
    end

    if flag_safe==1
        display(['2 succed 2'])
%         display(center_new)
    [param_safe2,param_best2] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,center_new,outline_all_negative,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    % 计算每个点到直线的距离
    end

end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





if isempty(outline_all_negative)
    display(['3'])
%     area = polyarea(obs_no_circle{pos_id}(1:end-1,1),obs_no_circle{pos_id}(1:end-1,2));
%     center = [mean(obs_no_circle{pos_id}(1:end-1,1)), mean(obs_no_circle{pos_id}(1:end-1,2))];
%     total_field=obtian_partfield(2*area,End_Point,outline_all,1.5,30,center);
    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,0.8*R,farthest_point_min,1,total_field,1);
    flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
    pose=1;
    while flag_safe==0
        pose=pose+1;
        if pose>poit_num
            break
        end
        center_new=obtain_new_center(a,b,c,Start_Point,0.8*R,farthest_point_min,1,total_field,pose);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);

    end



    if flag_safe==0
        display('type 2')
        center_new=obtain_new_center_new(a,b,c,Start_Point,R+1,farthest_point_min,1);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
        k=0;
        while flag_safe==0
            k=k+0.1;
            center_new=obtain_new_center_new(a,b,c,Start_Point,R+1-k,farthest_point_min,1);
            flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
            if k==R+1
                break
            end
        end
    end


    if flag_safe==1
        display(['3 succed 1'])
%         display(center_new)
    [param_safe1,param_best1] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,center_new,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    % 计算每个点到直线的距离
    end

%%%%%%%%%%%%%%%%%
distance_all_positive=distances(find(distances>0));
points_positive=points(find(distances>0),:);
[max_distance, max_index] = max(distance_all_positive);

farthest_point_max = points_positive(max_index, :);
    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_max,2,total_field,1);

    [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_max,2,total_field,poit_num);
    flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
    pose=poit_num;
    while flag_safe==0
        pose=pose-1;
        if pose<1
            break
        end
        center_new=obtain_new_center(a,b,c,Start_Point,-0.8*R,farthest_point_max,2,total_field,pose);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);


    end
    if flag_safe==0
        display('type 2')
        center_new=obtain_new_center_new(a,b,c,Start_Point,R+1,farthest_point_max,2);
        flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
        k=0;
        while flag_safe==0
            k=k+0.1;
            center_new=obtain_new_center_new(a,b,c,Start_Point,R+1-k,farthest_point_max,2);
            flag_safe=if_safe_point(obs_no_circle,center_new,r_e);
            if k==R+1
                break
            end
        end
    end

    if flag_safe==1
        display(['3 succed 2'])
%         display(center_new)
    [param_safe2,param_best2] = Dubins_obs_nocircle_oneside_step1(Start_Point,End_Point,center_new,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle);
    % 计算每个点到直线的距离
    end












end






% param_all=struct();
if exist('param_safe1', 'var') && isfield(param_safe1, 'pos_id') 
    if ~isempty(param_safe1)
%         display(['succed find   ',num2str(pos_id),'  up'])
    param_all=[param_safe1];
    end
    if exist('param_safe2', 'var') && isfield(param_safe2, 'pos_id') 
        if ~isempty(param_safe2)
%             display(['succed find   ',num2str(pos_id),'  up and down'])
        param_all=[param_all,param_safe2];
        end
    end  
elseif exist('param_safe2', 'var') && isfield(param_safe2, 'pos_id') 
    if ~isempty(param_safe2) 
%         display(['succed find   ',num2str(pos_id),'  down'])
        param_all=[param_safe2];
    end
else
%         display(['not find   ',num2str(pos_id),'  abondon'])


end


if ~exist('param_all', 'var')
%             display('not in end')
    param_all.path=[];
end





% param_all=[param_safe1,~isempty(param_safe2)*param_safe2];
param_best=[];
end





