function  Draw_map(startpoint,endpoint,obs,sure)
scatter(startpoint(1),startpoint(2),'*','r','LineWidth',1);axis equal;hold on
scatter(endpoint(2),endpoint(2),'*','r','LineWidth',1);
quiver(startpoint(1),startpoint(2),15*cos(startpoint(3)),15*sin(startpoint(3)),'Color','r','LineWidth',1);
quiver(endpoint(1),endpoint(2),15*cos(endpoint(3)),15*sin(endpoint(3)),'Color','r','LineWidth',1);

% scatter(param.aim(1),param.aim(2),'>','r','LineWidth',1);
text(startpoint(1),startpoint(2),['UAV']);
%    text(param.point(1,Le),param.point(2,Le),['UAV',num2str(i)]);
text(endpoint(2),endpoint(2),['TAR']);

num = length(obs(:,1));
for i=1:num
    th = 0:0.01:2*pi;
    edgex1 = obs(i,1) + (obs(i,3)-sure)*cos(th);
    edgey1 = obs(i,2) + (obs(i,3)-sure)*sin(th);
    plot(edgex1,edgey1,'LineStyle','--','Color','k');


    edgex2 = obs(i,1) + obs(i,3)*cos(th);
    edgey2 = obs(i,2) + obs(i,3)*sin(th);
    plot(edgex2,edgey2,'LineStyle','--','Color',[0,0.5,1]);

    plot(obs(i,1) , obs(i,2),'o');

end
title('MAP')

end