function [x,y]=obtain_circle(center_x,center_y,radius,num)
% 定义圆的参数
% center_x = 0; % 圆心的 x 坐标
% center_y = 0; % 圆心的 y 坐标
% radius = 1;   % 圆的半径

% 生成圆周上的点
theta = linspace(0, 2*pi, num); % 角度从0到2π，分100个点
x = center_x + radius * cos(theta); % x 坐标
y = center_y + radius * sin(theta); % y 坐标
end