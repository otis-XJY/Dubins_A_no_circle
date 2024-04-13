function [outline_all_unknow,obs_no_circle_unknow,obs_no_circle_in_unknow]=obtain_unknow_obs_nocircle(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in,final_path,num_steps)
% 创建新的坐标轴
figure
Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
Draw_pathA(final_path,sure);

% 初始化障碍物坐标和半径


outline_all_unknow =[];
obs_no_circle_unknow={};
obs_no_circle_in_unknow={};
k=0;
while true

    [x, y, button] = ginput(1);
%     disp('点击鼠标左键来添加障碍物，中键撤销上一个点，点击右键结束');
    
    % 左键为1，右键为3
    if button == 1
        % 添加障碍物
        k=k+1;
        % 获取键盘输入障碍物半径
%         display('请输入障碍物范围')
        prompt = '请输入障碍物半径：';
        R = input(prompt);

        
        center =[x,y];

        
        % 生成点的数量
        num_points = 10;
        
        % 生成随机半径
        % radius = (outer_radius - inner_radius) * rand(1, num_points) + inner_radius;
        radius = R*sqrt(rand(1, num_points));
        
        % 生成随机角度
        theta = 2*pi*rand(1, num_points);
        
        % 转换为笛卡尔坐标系
        x = center(1) + radius.*cos(theta);
        y = center(2) + radius.*sin(theta);
        
        % 组合 x 和 y 坐标形成点的集合
        points = [x; y]';
        
        
        
        
        
        
        
        
        
        
        % % 生成随机点
        % num_points = 10; % 点的数量
        % points = randi(mapsize,1,2)+randi(R,num_points,2); % 生成随机点坐标
        % points = 175+randi(R,num_points,2); % 生成随机点坐标
        % points = 150+randi(R,2, num_points); % 生成随机点坐标
        
        % 构建凸包
        boundary_points = convhull(points(:,1), points(:,2));
        % boundary_points = boundary(points(:,1), points(:,2));
        
        x=points(boundary_points,1);
        y=points(boundary_points,2);
        
        
        % x=points(1,k);
        % y=points(2,k);
        vertex=[x,y];
        
        
        expandedPolygon = expandshape(vertex, sure);
        
        x=expandedPolygon(:,1);
        y=expandedPolygon(:,2);
        
        % plot(x,y,'o')
        
        in1 = inpolygon(Start_Point(1),Start_Point(2),x,y);
        in2 = inpolygon(End_Point(1),End_Point(2),x,y);
        
        
        % num_steps=10;
        
        % 定义采样间隔
        % sampling_interval = 0.1;
        outline_all=[];
        if ~in1 && ~in2 
            for i=1:length(x)-1
                x_interp = linspace(x(i), x(i+1), num_steps);
                y_interp = linspace(y(i), y(i+1), num_steps);
                outline=[x_interp;y_interp]';
                outline_all=[outline_all;outline];
            end
        else
            display('sorry! 随机障碍包含了起点')
        end

        outline_all=[outline_all,ones(length(outline_all(:,1)),1)*k];

        outline_all_unknow =[outline_all_unknow;outline_all];
        obs_no_circle_unknow{end+1}=expandedPolygon;
        obs_no_circle_in_unknow{end+1}=vertex;


    elseif button == 2
        if ~isempty(obs_no_circle_unknow)
            % 撤销上一个点
            id=find(outline_all_unknow(:,3)==k);
            outline_all_unknow(id,:)=[];
            obs_no_circle_unknow{end}={};
            obs_no_circle_in_unknow{end}={};


%             % 清除图像
%             clf;
%             hold on;
%             axis equal;
%             % 重新绘制点
%             drawPoints(points);
        else
            display('no obs to delete')
        end


    elseif button == 3
        % 右键退出循环
        break;
    end
    
    clf;
%     figure;
    Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
    Draw_pathA(final_path,sure);
    grid on;
    hold on;
    axis equal;

    Draw_obs_unknow_circle(obs_no_circle_unknow,obs_no_circle_in_unknow)


end

end
