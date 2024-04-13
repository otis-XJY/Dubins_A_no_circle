function path_safe=point_in_path(obs_unknow,obs_range,path,point_num,now_id)

side_all=[];
path_safe_all=[];
plot(path(1,now_id:now_id+point_num),path(2,now_id:now_id+point_num),'.','Color','y')

if ~isempty(obs_range)
    points=obs_unknow(obs_range,:);
    
    
    % plot(points(:,1),points(:,2))
    % axis equal
    
    % 使用 DBSCAN 算法进行聚类
    epsilon = 3; % 邻域半径
    minPts = 10;    % 最小邻域点数
    [idx, centers] = dbscan(points, epsilon, minPts);
    
    % 输出聚类结果
    % disp("聚类结果：");
    % disp(idx);
    
    % 计算圆的个数（类别数）
    num_circles = length(unique(idx)); % -1是因为聚类算法中会将噪声点分配给 -1 类
    % disp("圆的个数：" + num_circles);
    
    
    for j=1:num_circles
        if isempty(find(idx==j))
            j=-1;
        end
    %     x=points(find(idx==j),1);
    %     y=points(find(idx==j),2);
    
        curve_points=(points(find(idx==j),:));
        
        for i=0:point_num
    
            if now_id+i>length(path)
                i=length(path)-now_id;
            end
        
            side=obtain_point_side(curve_points,path(:,now_id+i));
            side_all=[side_all,side];
        
        end
        
        side1=find(side_all==1);
        side2=find(side_all==2);
        
        if isempty(side1)||isempty(side2)
            path_safe_all=[path_safe_all,true];
        else
            path_safe_all=[path_safe_all,false];
        
        end
    end
    
    if all(path_safe_all==true)
        path_safe=true;
    else
        path_safe=false;
    end
else
    path_safe=true;
end



end
    
