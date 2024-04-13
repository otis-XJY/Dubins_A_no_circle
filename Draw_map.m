function  Draw_map(startpoint,endpoint,obs,sure,obs_no_circle,obs_no_circle_in)
scatter(startpoint(1),startpoint(2),'*','r','LineWidth',1);axis equal;hold on
scatter(endpoint(1),endpoint(2),'*','r','LineWidth',1);
quiver(startpoint(1),startpoint(2),15*cos(startpoint(3)),15*sin(startpoint(3)),'Color','r','LineWidth',1);
quiver(endpoint(1),endpoint(2),15*cos(endpoint(3)),15*sin(endpoint(3)),'Color','r','LineWidth',1);

% scatter(param.aim(1),param.aim(2),'>','r','LineWidth',1);
text(startpoint(1),startpoint(2),['UAV']);
%    text(param.point(1,Le),param.point(2,Le),['UAV',num2str(i)]);
text(endpoint(1),endpoint(2),['TAR']);

% % % % % % % % % % % % % line=[startpoint;endpoint];
% % % % % % % % % % % % % plot(line(:,1),line(:,2),'k--')
if ~isempty(obs)
    for i=1:length(obs(:,1))
        th = 0:0.01:2*pi;
        edgex1 = obs(i,1) + (obs(i,3)-sure)*cos(th);
        edgey1 = obs(i,2) + (obs(i,3)-sure)*sin(th);
        plot(edgex1,edgey1,'LineStyle','--','Color','k');
    
    
        edgex2 = obs(i,1) + obs(i,3)*cos(th);
        edgey2 = obs(i,2) + obs(i,3)*sin(th);
        plot(edgex2,edgey2,'LineStyle','--','Color',[0,0.5,1]);
    
        plot(obs(i,1) , obs(i,2),'o');
        text(obs(i,1),obs(i,2),num2str(i))
    
    end
end




for i=1:length(obs_no_circle)
    if ~isempty(obs_no_circle{i})
      x=obs_no_circle{i}(:,1);
      y=obs_no_circle{i}(:,2);
      plot(x,y,'b-','LineWidth',2)
      text(mean(x),mean(y),num2str(i))
    end
end

for i=1:length(obs_no_circle_in)
    if ~isempty(obs_no_circle_in{i})
      x=obs_no_circle_in{i}(:,1);
      y=obs_no_circle_in{i}(:,2);
      plot(x,y,'r-','LineWidth',2)
%       text(mean(x),mean(y),num2str(i))
    end
end
title('MAP')

end