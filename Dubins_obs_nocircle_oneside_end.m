function [param_safe,param_best] = Dubins_obs_nocircle_oneside_end(Start_Point,End_Point,outline_all_positive,outline,r_s,r_e,R,Stepsize,pos_id,parent_id,obs_no_circle,Start_point_now,param_safe)        
        param_best={};
        param_final=[];
%         display('try end')
        points=outline_all_positive(:,1:2);
        x0=Start_point_now(1);
        y0=Start_point_now(2);%%%%直线上一点
        m=(End_Point(2)-Start_point_now(2))/(End_Point(1)-Start_point_now(1));%%%%斜率
        
        a=m;
        b=-1;
        c=y0-m*x0;
        
        % 生成直线上的点
        x = linspace(100, 200, 100);  % 生成 x 坐标的范围
        y = (-a*x - c) / b;          % 根据直线方程计算对应的 y 坐标
        
        % 绘制直线
        % plot(x, y, 'b-', 'LineWidth', 2);
        
        % 设置坐标轴范围
        % axis equal;
        
        
        % 计算每个点到直线的距离
        singal= (a*End_Point(1) + b*End_Point(2) + c) / sqrt(a^2 + b^2);
        distances = (a*points(:,1) + b*points(:,2) + c) / sqrt(a^2 + b^2);
        
        % 找到距离最大的点及其索引
        if singal>0
            [max_distance, max_index] = max(distances);
            farthest_point = points(max_index, :);
            outline_all_positive=outline_all_positive(find(distances>=0),:);

        else
            [min_distance, min_index] = min(distances);
            farthest_point = points(min_index, :);
            outline_all_positive=outline_all_positive(find(distances<0),:);
        end
        if ~isempty(farthest_point)
%         display('try oneside in end')
        [param_all,param_best] = Dubins_obs_nocircle_oneside(Start_point_now,End_Point,farthest_point,outline_all_positive,outline,r_s,R,R,Stepsize,pos_id,parent_id,obs_no_circle);
        for i=1:length(param_safe)
            for j=1:length(param_all)
                
                
                if isfield(param_safe(i), 'pos_id') && isfield(param_all(j), 'pos_id')
                if ~isempty(param_safe(i).pos_id) && ~isempty(param_all(j).pos_id) && all(isnan(param_all(j).length)==0)
%                 display('succed end')
                param_final(j+i).point=[param_safe(i).point(:,1:2),param_all(j).point];
                param_final(j+i).length=[param_safe(i).length(1:2),param_all(j).length];
                param_final(j+i).phy=[param_safe(i).phy(1),param_all(j).phy];
                param_final(j+i).theta=[param_safe(i).theta(1:2),param_all(j).length];
                param_final(j+i).center=[param_safe(i).center(:,1),param_all(j).center];
%                 param_final(j+i).center=[param_safe(i).center(:,1),param_all(j).center(:,1)];
                param_final(j+i).r=[r_s,r_e];
                param_final(j+i).R=R;
                param_final(j+i).type=-1;
                param_final(j+i).vtheta=[param_safe(i).vtheta(1:2),param_all(j).vtheta];
                param_final(j+i).Length=sum(param_final(j+i).length);
                param_final(j+i).path=[param_safe(i).path(1:2),param_all(j).path];
                param_final(j+i).vtheta_all=[param_safe(i).vtheta_all(1:2),param_all(j).vtheta_all];
                param_final(j+i).pos_id=pos_id;
                param_final(j+i).parent_id=parent_id;
                end
                end
            end
        end
        param_safe=param_final;
        end


        if ~exist('param_safe', 'var')
%             display('not in end')
            param_safe=struct();
        end
end