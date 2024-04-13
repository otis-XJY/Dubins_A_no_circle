function drawcircle_apf(center_x,center_y,radius,num)
% 定义圆的参数
% center_x = 0; % 圆心的 x 坐标
% center_y = 0; % 圆心的 y 坐标
% radius = 1;   % 圆的半径

% 生成圆周上的点
theta = linspace(0, 2*pi, num); % 角度从0到2π，分100个点
x = center_x + radius * cos(theta); % x 坐标
y = center_y + radius * sin(theta); % y 坐标

% 绘制圆
plot(x, y, 'b'); % 使用蓝色线条绘制圆
axis equal; % 设置坐标轴比例相等，保证圆形显示
xlabel('X'); % 添加 X 轴标签
ylabel('Y'); % 添加 Y 轴标签
title('Circle Plot'); % 添加图表标题
plot(center_x,center_y,'o')
grid on; % 显示网格
hold on
end