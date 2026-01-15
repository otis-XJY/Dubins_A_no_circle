function  DrawD(param)
Le = length(param.point(1,:));
plot(param.path(1,:),param.path(2,:),'LineWidth',1);axis equal;hold on
scatter(param.point(1,1),param.point(2,1),'*','r','LineWidth',1);
plot(param.point(1,2),param.point(2,2),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
plot(param.point(1,3),param.point(2,3),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
plot(param.point(1,4),param.point(2,4),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
plot(param.point(1,5),param.point(2,5),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
scatter(param.point(1,Le),param.point(2,Le),'*','r','LineWidth',1);

quiver(param.point(1,1),param.point(2,1),10*cos(param.vtheta(1)),10*sin(param.vtheta(1)),'Color','r','LineWidth',1);
%    scatter(param.point(1,Le),param.point(2,Le),'*','r','LineWidth',1);
% line([param.point(1,Le),param.aim(1)],[param.point(2,Le),param.aim(2)],'LineStyle',':','LineWidth',1.5);
quiver(param.point(1,Le),param.point(2,Le),10*cos(param.vtheta(2)),10*sin(param.vtheta(2)),'Color','r','LineWidth',1);

% scatter(param.aim(1),param.aim(2),'>','r','LineWidth',1);
text(param.point(1,1), param.point(2,1),['UAV']);
%    text(param.point(1,Le),param.point(2,Le),['UAV',num2str(i)]);
text(param.point(1,Le),param.point(2,Le),['TAR']);

th = 0:0.01:2*pi;
edge1x = param.center(1,1) + param.r*cos(th);
edge1y = param.center(2,1) + param.r*sin(th);
edge2x = param.center(1,2) + param.R*cos(th);
edge2y = param.center(2,2) + param.R*sin(th);
edge3x = param.center(1,3) + param.r*cos(th);
edge3y = param.center(2,3) + param.r*sin(th);
plot(param.center(1,1),param.center(2,1),'o');
plot(param.center(1,2),param.center(2,2),'o');
plot(param.center(1,3),param.center(2,3),'o');
plot(edge1x,edge1y,'LineStyle','--','Color','g');
plot(edge2x,edge2y,'LineStyle','--','Color','k');
plot(edge3x,edge3y,'LineStyle','--','Color','g');

end