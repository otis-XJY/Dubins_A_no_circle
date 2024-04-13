function draw_line(slope,x0,y0,xmin,xmax)
% 计算直线的截距
intercept = y0 - slope * x0;

% 定义 x 的范围
x = linspace(xmin, xmax, 100);  % 从 -10 到 10 均匀取 100 个点

% 计算对应的 y 坐标值
y = slope * x + intercept;

% 绘制直线
plot(x, y, '-.', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
title('MAP');
axis equal;

end