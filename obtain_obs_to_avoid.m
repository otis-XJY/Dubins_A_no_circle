function obs_to_avoid=obtain_obs_to_avoid(Start_Point,End_Point,outline_all)
x0=Start_Point(1);
y0=Start_Point(2);%%%%直线上一点
m=(End_Point(2)-Start_Point(2))/(End_Point(1)-Start_Point(1));%%%%斜率

a=m;
b=-1;
c=y0-m*x0;


k_vertical=b/a;
a1=k_vertical;
b1=-1;
c1=Start_Point(2)-k_vertical*Start_Point(1);

x=Start_Point(1):0.5:End_Point(1);

% y1=(-a1*x-c1)/b1;
% y2=(-a*x-c)/b;
% 
% plot(x,y1,'r-');hold on;axis equal
% plot(x,y2,'b-')




distances = (a1*outline_all(:,1) + b1*outline_all(:,2) + c1) / sqrt(a1^2 + b1^2);

singal=(a1*End_Point(1) + b1*End_Point(2) + c1) / sqrt(a1^2 + b1^2);

if singal>0
    obs_id_all=outline_all(find(distances>=0),3);
else
    obs_id_all=outline_all(find(distances<0),3);
end
[obs_to_avoid, ~, ~] = unique(obs_id_all);
