function [center_all,vertex,ind]=obtain_obs_nocircle2circle(Start_Point,End_Point,mapsize,R,safe)
center_all=[];
%% vertex 顶点数目
%  num_steps轮廓的采样率







% 生成随机点
num_points = 10; % 点的数量
points = randi(mapsize,2,1)+randi(R,2, num_points); % 生成随机点坐标
% points = 150+randi(R,2, num_points); % 生成随机点坐标

% 构建凸包
k = convhull(points(1,:), points(2,:));

x=points(1,k);
y=points(2,k);
vertex=[x;y];



in1 = inpolygon(Start_Point(1),Start_Point(2),x,y);
in2 = inpolygon(End_Point(1),End_Point(2),x,y);
% if in
%     disp('点在多边形内部');
% else
%     disp('点在多边形外部');
% end

ind={};
if ~in1 && ~in2   
    for i=1:length(x)-1
        [center,outline]=obtain_line_circle(x(i+1),x(i),y(i+1),y(i),safe);
        ind{i}=outline;
        center_all=[center_all;center];
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