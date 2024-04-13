function isIntersect = isPolygonsIntersect(polygon1, polygon2)
    % 判断两个多边形是否相交
    % 输入参数：
    %   polygon1: 第一个多边形的顶点，一个 Nx2 的矩阵，每行代表一个顶点的坐标
    %   polygon2: 第二个多边形的顶点，一个 Mx2 的矩阵，每行代表一个顶点的坐标
    % 输出参数：
    %   isIntersect: 逻辑值，表示两个多边形是否相交

    % 检查边界相交或点在多边形内
    isIntersect = ~isempty(intersect(polygon1, polygon2)) || any(inpolygon(polygon1(:,1), polygon1(:,2), polygon2(:,1), polygon2(:,2))) || any(inpolygon(polygon2(:,1), polygon2(:,2), polygon1(:,1), polygon1(:,2)));
end
