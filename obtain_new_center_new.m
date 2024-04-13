function [center_new,poit_num]=obtain_new_center_new(a,b,c,Start_Point,distance,center_old,type)


%%%%已知直线a,b,c
%%%%将center_old沿垂直于该直线的方向=>移动distance
%%%%type=1方向为靠近已知直线
%%%%type=2方向为远离已知直线



% % % % k_vertical=-1/a;
% % % % 
% % % % 
% % % % a1=k_vertical;
% % % % b1=-1;
% % % % c1=center_old(2)-k_vertical*center_old(1);
% % % % 
% % % % x_end = (b1*c - b*c1) / (a1*b - a*b1);
% % % % y_end = (a*c1 - a1*c) / (a1*b - a*b1);
% % % % 
% % % % % center_old
% % % % 
% % % % if x_end>center_old(1)
% % % %     x=center_old(1):0.5:x_end;
% % % % else
% % % %     x=x_end:0.5:center_old(1);
% % % % end
% % % % 
% % % % 
% % % % 
% % % % % x=Start_Point(1):0.5:center_old(1);
% % % % % y_range=Start_Point(2):0.5:center_old(2);
% % % % y=(-a1*x-c1)/b1;
% % % % 
% % % % field=[];
% % % % [x;y]
% % % % 
% % % % 
% % % % for i=1:length(x)
% % % % %     total_field(ceil(x(i)/0.5+1),ceil(y(i)/0.5+1))
% % % %     field=[field,total_field(ceil(x(i)/0.5+1),ceil(y(i)/0.5+1))];
% % % % end
% % % % 
% % % % 
% % % % field_sort=sort(field);
% % % % id=find(field==field_sort(pos));
% % % % 
% % % % 
% % % % poit_num=length(field);
% % % % center_new=[x(id),y(id)];







singal = (a*center_old(1) + b*center_old(2) + c) / sqrt(a^2 + b^2);

k_vertical=-1/a;

norm_vertical1=[1,k_vertical] / norm([1, k_vertical]);
norm_vertical2=-norm_vertical1;

displacement1 = distance * norm_vertical1;
new_point1 = Start_Point(1:2) + displacement1;
distances1 = (a*new_point1(1) + b*new_point1(2) + c) / sqrt(a^2 + b^2);

displacement2 = distance * norm_vertical2;
new_point2 = Start_Point(1:2) + displacement2;
% distances2 = (a*new_point2(1) + b*new_point2(2) + c) / sqrt(a^2 + b^2);


if type==1

if singal*distances1>=0
    center_new=center_old + displacement2;
else
    center_new=center_old + displacement1;
end

end


if type==2

if singal*distances1>=0
    center_new=center_old + displacement1;
else
    center_new=center_old + displacement2;
end

end

