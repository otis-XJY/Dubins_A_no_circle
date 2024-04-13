function [center_new,poit_num]=obtain_new_center(a,b,c,Start_Point,R,center_old,type,total_field,pos,boundary)


%%%%已知直线a,b,c
%%%%将center_old沿垂直于该直线的方向=>移动distance
%%%%type=1方向为靠近已知直线
%%%%type=2方向为远离已知直线





k_vertical=b/a;


a1=k_vertical;
b1=-1;
c1=center_old(2)-k_vertical*center_old(1);

if type==1
    if abs(c)~=inf
    x_end = (b1*c - b*c1) / (a1*b - a*b1);
    y_end = (a*c1 - a1*c) / (a1*b - a*b1);
    else
    x_end=Start_Point(1);
    y_end=c1;
    end
    

else
%     x_test=[0,length(total_field)];
%     y_test=[0,length(total_field)];

%     mostx=max(boundary(1,:));
%     mosty=max(boundary(2,:));
    
    
%     most=(length(total_field)-1);
    most=753;%%mapsize


    points_test=[0,(-a1*0-c1)/b1;
                 most,(-a1*most-c1)/b1;
                 (-c1-b1*0)/a1,0;
                 (-c1-b1*most)/a1,most];
    id1=find(points_test(:,1)>most);
    id2=find(points_test(:,1)<0);
    id3=find(points_test(:,2)>most);
    id4=find(points_test(:,2)<0);
    points_test([id1,id2,id3,id4],:)=[];
%     points_test
 if abs(c)~=inf
    distances = (a*points_test(:,1) + b*points_test(:,2) + c) / sqrt(a^2 + b^2);
    signal=(a*center_old(1) + b*center_old(2) + c) / sqrt(a^2 + b^2);
 else
    distances=points_test(:,1)-Start_Point(1);
    signal=center_old(1)-Start_Point(1);

 end
    % center_old


    if signal>0
        x_end=points_test(find(distances>0),1);
    else
        x_end=points_test(find(distances<0),1);
    end


end
    % center_old
    resoultion=1.4;
    if x_end>center_old(1)
        x=center_old(1):resoultion:x_end;
    else
        x=x_end:resoultion:center_old(1);
    end
    
    
    
    % x=Start_Point(1):0.5:center_old(1);
    % y_range=Start_Point(2):0.5:center_old(2);
    y=(-a1*x-c1)/b1;
    
    field=[];
    % [x;y]
%     x_end
%     center_old
    
    for i=1:length(x)
    %     total_field(ceil(x(i)/0.5+1),ceil(y(i)/0.5+1))
        if ceil(x(i)/resoultion)+1>0 && ceil(y(i)/resoultion)+1>0 && ...
                ceil(x(i)/resoultion)+1<=length(total_field) && ceil(y(i)/resoultion)+1<=length(total_field)
            field=[field,total_field(ceil(x(i)/resoultion)+1,ceil(y(i)/resoultion)+1)];
        end
    end
    
    
    field_sort=sort(field);
    id=find(field==field_sort(pos));
    
    
    poit_num=length(field);
    center_new=[x(id),y(id)];


distance=sqrt((x(id)-center_old(1))^2+(y(id)-center_old(2))^2)-R;




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

