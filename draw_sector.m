function draw_sector(angle,fov,pos,radius,obs_range,obs)
% 
% x=-5;
% y=-0.2;
% angle=atan2(y,x);

% fov=84;%%%°
% 定义扇形的参数
angle_start=angle-fov/2*pi/180;
angle_end=angle+fov/2*pi/180;
% center = [0, 0]; % 扇形的中心点坐标
% radius = 5; % 扇形的半径
theta = linspace(angle_start, angle_end, 100); % 扇形的角度范围

% 计算扇形的边界点
x = pos(1) + radius * cos(theta);
y = pos(2) + radius * sin(theta);

if ~isempty(obs_range)
    for i=1:length(obs_range)
        plot(obs(obs_range(i),1),obs(obs_range(i),2),'or')
    end
end


% 绘制扇形
quiver(pos(1),pos(2),10*cos(angle),10*sin(angle),'ok');hold on;
plot(x,y)
plot([pos(1), x(1)], [pos(2), y(1)], 'b', 'LineWidth', 2);
plot([pos(1), x(end)], [pos(2), y(end)], 'b', 'LineWidth', 2);
axis equal; % 设置坐标轴比例相等，确保圆形显示
% title('Sector Plot');
xlabel('X');
ylabel('Y');
end