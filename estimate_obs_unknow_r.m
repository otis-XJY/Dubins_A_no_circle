function r_max=estimate_obs_unknow_r(obs_range,obs_all,X_now_APF)

points=obs_all(obs_range,:);

distance1=sqrt((points(:,1)-X_now_APF(1)).^2+(points(:,2)-X_now_APF(2)).^2);

r_max1=max(distance1)-min(distance1);

distance2=sqrt((max(points(:,1))-min(points(:,1))).^2+(max(points(:,2))-min((points(:,2)))).^2);

r_max2=distance2;

r_max=max([r_max1,r_max2]);





















% % % % % 假设你有一系列点存储在一个二维数组 points 中，每一行表示一个点的坐标 (x, y)
% % % % R = 5;
% % % % th = 0:0.01:2*pi;
% % % % edgex1 = 0 + 7*cos(th);
% % % % edgey1 = 0 + 7*sin(th);
% % % % edgex2 = 3 + 9*cos(th);
% % % % edgey2 = 5 + 9*sin(th);
% % % % 
% % % % edgex3 = 3 + 15*cos(th);
% % % % edgey3 = 5 + 15*sin(th);
% % % % 
% % % % points=[];
% % % % points(:,1)=[edgex1(1:10),edgex2(85:100),edgex3(5:10)];
% % % % points(:,2)=[edgey1(1:10),edgey2(85:100),edgey3(5:10)];
% % % 
% % % points=obs_all(obs_range,:);
% % % 
% % % 
% % % % plot(points(:,1),points(:,2))
% % % % axis equal
% % % 
% % % % 使用 DBSCAN 算法进行聚类
% % % epsilon = 1; % 邻域半径
% % % minPts = 5;    % 最小邻域点数
% % % [idx, centers] = dbscan(points, epsilon, minPts);
% % % 
% % % % 输出聚类结果
% % % % disp("聚类结果：");
% % % % disp(idx);
% % % 
% % % % 计算圆的个数（类别数）
% % % num_circles = length(unique(idx)); % -1是因为聚类算法中会将噪声点分配给 -1 类
% % % % disp("圆的个数：" + num_circles);
% % % 
% % % 
% % % r_all=[];
% % % for i=1:num_circles
% % %     if isempty(find(idx==i))
% % %         i=-1;
% % %     end
% % %     x=points(find(idx==i),1);
% % %     y=points(find(idx==i),2);
% % %     A = [x(:), y(:), ones(size(x(:)))];
% % %     b = -x(:).^2 - y(:).^2;
% % %     coeff = A\b;
% % %     
% % %     % 从拟合系数中提取圆的半径和圆心坐标
% % %     center_x = -coeff(1) / 2;
% % %     center_y = -coeff(2) / 2;
% % %     radius = sqrt((center_x^2 + center_y^2) - coeff(3));
% % %     r_all=[r_all,radius];
% % %     
% % % %     % 绘制原始数据点和拟合圆
% % % %     theta = linspace(0, 2*pi, 100);
% % % %     circle_x = center_x + radius * cos(theta);
% % % %     circle_y = center_y + radius * sin(theta);
% % % %     
% % % %     plot(x, y, 'bo');  % 绘制原始数据点
% % % %     hold on;
% % % %     plot(circle_x, circle_y, 'r-');  % 绘制拟合圆
% % % %     axis equal;  % 设置坐标轴比例相等
% % % end
% % % 
% % % r_max=max(r_all);
% % % 
% % % end
% % % 
