function [center,outline]=obtain_line_circle(x2,x1,y2,y1,safe_r)

center_x=[];
center_y=[];

distance=sqrt((x2-x1)^2+(y2-y1)^2);
angle=atan2(y2-y1,x2-x1);

center=[];
if mod(distance,2*safe_r)
    point_x=linspace(x1,x2,ceil(distance/(2*safe_r)));
    point_y=linspace(y1,y2,ceil(distance/(2*safe_r)));
    center_x=[point_x(1)+safe_r*cos(angle),point_x(end)-safe_r*cos(angle)];
    center_y=[point_y(1)+safe_r*sin(angle),point_y(end)-safe_r*sin(angle)];

    if length(point_x)>2
        center_x=[center_x(1),point_x(2:length(point_x)-1),center_x(end)];
        center_y=[center_y(1),point_y(2:length(point_y)-1),center_y(end)];
    end

else
    point_x=linspace(x1,x2,ceil(distance/(2*safe_r))+1);
    point_y=linspace(y1,y2,ceil(distance/(2*safe_r))+1);
    for j=1:length(point_x)-1
        center_x=[center_x;(point_x(j+1)+point_x(j))/2];
        center_y=[center_y;(point_y(j+1)+point_y(j))/2];
    end
end



outline=[point_x;point_y];
%%%%%1,2
%%%%%1,3
%%%%(1,1)  (2,3)

center(:,1)=center_x;
center(:,2)=center_y;