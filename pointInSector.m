function isInSector = pointInSector(x_p, y_p, x_c, y_c, r, angle, fov)
theta1=angle-fov/2*pi/180;
theta2=angle+fov/2*pi/180;

% 计算点到扇形中心的距离
    d = sqrt((x_p - x_c)^2 + (y_p - y_c)^2);
    
    % 如果距离大于半径，则点不在扇形内
    if d >= r
        isInSector = false;
        return;
    end
    
    % 计算点和扇形中心构成的线段与 x 轴的夹角
    phi = atan2(y_p - y_c, x_p - x_c);
    
%     quiver(x_c,y_c,10*cos(phi),10*sin(phi),'r');hold on;
    
    
    % 调整夹角到 [0, 2*pi] 的范围
%     if phi < 0
%         phi = phi + 2*pi;
%     end
    
    % 调整起始角度和结束角度到 [0, 2*pi] 的范围
%     if theta1 < 0
%         theta1 = theta1 + 2*pi;
%     end
%     if theta2 < 0
%         theta2 = theta2 + 2*pi;
%     end
%     quiver(x_c,y_c,10*cos(theta1),10*sin(theta1),'r');hold on;
%     quiver(x_c,y_c,10*cos(theta2),10*sin(theta2),'r');hold on;
%     display([phi,theta1,theta2])
    % 判断点是否在扇形角度范围内
    if theta1 < phi && phi < theta2
        isInSector = true;
    else
        isInSector = false;
    end
end
