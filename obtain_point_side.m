function side=obtain_point_side(curve_points,point)

% 曲线表示为一系列点
% curve_points=[];
% th = 0:0.01:2*pi;
% edgex1 = 0 + 7*cos(th);
% edgey1 = 0 + 7*sin(th);
% point = [5 5]; % 要判断的点
% curve_points(:,1)=edgex1(1:100);
% curve_points(:,2)=edgey1(1:100);
% 
% plot(curve_points(:,1),curve_points(:,2),'.','Color','r');hold on; axis equal
% plot(point(1),point(2),'o')

% 计算点到曲线的距离
distances = sqrt((curve_points(:,1) - point(1)).^2 + (curve_points(:,2) - point(2)).^2);

% 找到最近的点的索引
[min_distance, min_index] = min(distances);



if min_index == length(curve_points)||min_index==1
%     vector1 = curve_points(min_index, :) - curve_points(min_index-1, :);
    side=0;
else
    vector1 = curve_points(min_index + 1, :) - curve_points(min_index, :);
    vector2 = curve_points(min_index, :)-point';

    cross_product = vector2(1) * vector1(2) - vector2(2) * vector1(1);
    
    
    if cross_product > 0
        side = 1;%%%%逆时针
    else
        side = 2;%%%顺时针
    end
end


end

