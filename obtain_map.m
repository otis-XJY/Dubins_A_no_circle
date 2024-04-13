function [obs,obs_no_circle,obs_no_circle_in,outline_all]=obtain_map(num,mapsize,Start_Point,End_Point,R,r,obs_no_circle,obs_no_circle_in,outline_all)

obs=[round(rand(num,2)*mapsize,2),randi(R,num,1)];%[x,y,障碍物的半径R]

removeInd=[];
for i=1:length(obs(:,1))
    dis_start=sqrt((Start_Point(1)-obs(i,1))^2+(Start_Point(2)-obs(i,2))^2);
    dis_end=sqrt((End_Point(1)-obs(i,1))^2+(End_Point(2)-obs(i,2))^2);    
    if dis_start<(obs(i,3)+2*r) || dis_end<(obs(i,3)+2*r)
        removeInd=[removeInd;i];
    end
end
obs(removeInd,:)=[];



removeInd=[];


% 定义正方形边界的范围
x_min_start = Start_Point(1)-2*r;
x_max_start = Start_Point(1)+2*r;
y_min_start = Start_Point(2)-2*r;
y_max_start = Start_Point(2)+2*r;

% 定义点的间距
spacing = 0.1;

% 生成点的坐标
[x_start, y_start] = meshgrid(x_min_start:spacing:x_max_start, y_min_start:spacing:y_max_start);

x_start = x_start(:);
y_start = y_start(:);
% plot(x_start,y_start,'x');hold on
% 定义正方形边界的范围
x_min_end = End_Point(1)-2*r;
x_max_end =End_Point(1)+2*r;
y_min_end =End_Point(2)-2*r;
y_max_end =End_Point(2)+2*r;


% 生成点的坐标
[x_end, y_end] = meshgrid(x_min_end:spacing:x_max_end, y_min_end:spacing:y_max_end);
x_end = x_end(:);
y_end = y_end(:);
% plot(x_end,y_end,'x')
ll=length(obs_no_circle);
for j=1:length(obs_no_circle)
    if ~isempty(obs_no_circle{j})
      x=obs_no_circle{j}(:,1);
      y=obs_no_circle{j}(:,2);
      in1 = inpolygon(x_start,y_start, x, y);
      in2= inpolygon(x_end,y_end, x, y);
      if j+1<=ll
          for k=j+1:ll
              if ~isempty(obs_no_circle{k})
                   in3=isPolygonsIntersect(obs_no_circle{j},obs_no_circle{k});
                   if in3==1
                       break
                   end
              end
          end
      end
      if ~all(in1==0) || ~all(in2==0) || in3==1
%           removeInd=[removeInd,j];
          obs_no_circle(j)={{}};
          obs_no_circle_in(j)={{}};
          id=find(outline_all(:,3)==j);
            outline_all(id,:)=[];
      end

      
    end
end
% if ~isempty(removeInd)
%     obs_no_circle(removeInd)={{}};
%     obs_no_circle_in(removeInd)={{}};
%     for i=1:length(removeInd)
%         id=find(outline_all(:,3)==removeInd(i));
%         outline_all(id,:)=[];
%     
%     end
% end




% for j=1:length(obs_no_circle_in)
%     if ~isempty(obs_no_circle_in{j})
%       x=obs_no_circle_in{j}(:,1);
%       y=obs_no_circle_in{j}(:,2);
%       in1 = inpolygon(x_start,y_start, x, y);
%       in2= inpolygon(x_end,y_end, x, y);
%       if ~all(in1==0) || ~all(in2==0)
%           removeInd=[removeInd,j];
%       end
%     end
% end
% if ~isempty(removeInd)
%     obs_no_circle_in(removeInd)={{}};
%     for i=1:length(removeInd)
%         id=find(outline_all(:,3)==removeInd(i));
%         outline_all(id,:)=[];
%     
%     end
% end



for j=1:length(obs_no_circle)
    if ~isempty(obs_no_circle{j})
      x=obs_no_circle{j}(:,1);
      y=obs_no_circle{j}(:,2);
      if ~isempty(find(x<10)) || ~isempty(find(y<10))
          removeInd=[removeInd,j];
      end
    end
end
if ~isempty(removeInd)
    obs_no_circle(removeInd)={{}};
    obs_no_circle_in(removeInd)={{}};
    for i=1:length(removeInd)
        id=find(outline_all(:,3)==removeInd(i));
        outline_all(id,:)=[];
    
    end
end



