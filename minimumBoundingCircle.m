function [center, radius] = minimumBoundingCircle(vertices)
    % 计算多边形的外接圆
    % 输入参数：
    %   vertices: 多边形的顶点，一个 Nx2 的矩阵，每行代表一个顶点的坐标
    % 输出参数：
    %   center: 外接圆的圆心坐标，一个 1x2 的向量
    %   radius: 外接圆的半径

    % 计算凸包
    k = convhull(vertices(:,1), vertices(:,2));
    convexHullVertices = vertices(k, :);

    % 计算凸包的直径
    [diameter, ~] = pdist2(convexHullVertices, convexHullVertices, 'euclidean', 'Largest', 2);
    diameter = max(diameter);
    radius = max(diameter / 2);

    % 计算凸包的重心作为外接圆的圆心
    center = mean(convexHullVertices);
end
