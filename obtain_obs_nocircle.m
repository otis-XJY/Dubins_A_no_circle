function [outline_all,vertex,expandedPolygon]=obtain_obs_nocircle(mapsize,R,num_steps,Start_Point,End_Point,sure)

%% vertex 顶点数目
%  num_steps轮廓的采样率

% center_all=[];
%% vertex 顶点数目
%  num_steps轮廓的采样率






% 圆心坐标
% center = [0, 0];
center =randi(mapsize,2,1);

% center =[100;100];

r=randi(R,1);

% 生成点的数量
num_points = 10;

% 生成随机半径
% radius = (outer_radius - inner_radius) * rand(1, num_points) + inner_radius;
radius = r*sqrt(rand(1, num_points));

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
end

end


%%%ind为轮廓值

% figure;
% 
% for i=1:length(ind)
%     plot(ind{i}(1,:), ind{i}(2,:), 'b', 'LineWidth', 2);hold on
% end
% axis equal; % 让 x 和 y 轴的比例相同
% title('不规则多边形轮廓插值');