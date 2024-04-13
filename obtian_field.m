function total_field=obtian_field(map_size,End_Point,outline_all,attractive_force,repulsive_force,resolution)

% 定义全局地图的大小和分辨率
% map_size = [100, 100]; % 地图大小为 100x100
map_size=[map_size,map_size];
% resolution = 0.5; % 分辨率为 0.5

% 定义起点、终点和障碍物位置
% start = Start_Point(1:2); % 起点位置
goal = End_Point(1:2); % 终点位置
% obstacles = [25, 25; 30, 30; 35, 35]; % 障碍物位置，示例中有三个障碍物
obstacles=[];
if ~isempty(outline_all)
    obstacles=outline_all(1:10:end,1:2);
end



[X, Y] = meshgrid(0:resolution:map_size(1), 0:resolution:map_size(2));

% 计算吸引力场
% attractive_force = 1.5;
distance_to_goal = sqrt((X - goal(1)).^2 + (Y - goal(2)).^2);
attractive_field = attractive_force * distance_to_goal;

% 初始化势场
potential_field = zeros(length(X));


% 计算斥力场
% repulsive_force = 30;
for i = 1:size(obstacles, 1)
    distance_to_obstacle = sqrt((X - obstacles(i, 1)).^2 + (Y - obstacles(i, 2)).^2);
    repulsive_field = repulsive_force ./ distance_to_obstacle;
    potential_field = potential_field + repulsive_field;
end

% 合并吸引力场和斥力场
total_field = attractive_field + potential_field;

% 绘制势场图
figure;
surf(X, Y, total_field,'FaceColor','flat');
% colormap('jet');
% 
% % 调整光照效果为 'gouraud'
% shading('flat');
% 
% % 添加光源，使图形更明亮
% lighting('gouraud');
% % light('Position', [1 0 1], 'Style', 'infinite');

