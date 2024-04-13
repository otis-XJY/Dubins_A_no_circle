function  DrawD_bset(param,sure)
Le = length(param.point(1,:));
scatter(param.point(1,1),param.point(2,1),'*','r','LineWidth',1);
scatter(param.point(1,Le),param.point(2,Le),'*','r','LineWidth',1);
axis equal;hold on
if Le==6
    for i=2:Le-1
        plot(param.point(1,i),param.point(2,i),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
    end
    quiver(param.point(1,3),param.point(2,3),15*cos(param.vtheta(2)),15*sin(param.vtheta(2)),'Color','r','LineWidth',1);
    quiver(param.point(1,2),param.point(2,2),15*cos(param.vtheta(2)),15*sin(param.vtheta(2)),'Color','r','LineWidth',1);
    quiver(param.point(1,4),param.point(2,4),15*cos(param.vtheta(3)),15*sin(param.vtheta(3)),'Color','r','LineWidth',1);
    quiver(param.point(1,5),param.point(2,5),15*cos(param.vtheta(3)),15*sin(param.vtheta(3)),'Color','r','LineWidth',1);
    quiver(param.point(1,Le),param.point(2,Le),15*cos(param.vtheta(4)),15*sin(param.vtheta(4)),'Color','r','LineWidth',1);
    for j=1:Le-1
        plot(param.path{j}(1,:),param.path{j}(2,:),'LineWidth',1);
    end
    th = 0:0.01:2*pi;
    edge1x = param.center(1,1) + param.r(1)*cos(th);
    edge1y = param.center(2,1) + param.r(1)*sin(th);
    edge2x = param.center(1,2) + param.R*cos(th);
    edge2y = param.center(2,2) + param.R*sin(th);
    edge4x = param.center(1,2) + (param.R-sure)*cos(th);
    edge4y = param.center(2,2) + (param.R-sure)*sin(th);
    edge3x = param.center(1,3) + param.r(2)*cos(th);
    edge3y = param.center(2,3) + param.r(2)*sin(th);
    plot(param.center(1,1),param.center(2,1),'o');
    plot(param.center(1,2),param.center(2,2),'o');
    plot(param.center(1,3),param.center(2,3),'o');
    plot(edge1x,edge1y,'LineStyle','--','Color','b');
    plot(edge2x,edge2y,'LineStyle','--','Color',[0,0.5,1]);
    plot(edge4x,edge4y,'LineStyle','--','Color','k');
    plot(edge3x,edge3y,'LineStyle','--','Color','b');

else
    for i=2:Le-1
        plot(param.point(1,i),param.point(2,i),'.','MarkerSize',15,'MarkerFaceColor','k','MarkerEdgeColor','k');
    end
    quiver(param.point(1,2),param.point(2,2),15*cos(param.vtheta(2)),15*sin(param.vtheta(2)),'Color','r','LineWidth',1);
    quiver(param.point(1,3),param.point(2,3),15*cos(param.vtheta(2)),15*sin(param.vtheta(2)),'Color','r','LineWidth',1);
    quiver(param.point(1,Le),param.point(2,Le),15*cos(param.vtheta(3)),15*sin(param.vtheta(3)),'Color','r','LineWidth',1);
    for j=1:Le-1
        plot(param.path{j}(1,:),param.path{j}(2,:),'LineWidth',1);
    end
    th = 0:0.01:2*pi;
    edge1x = param.center(1,1) + param.r(1)*cos(th);
    edge1y = param.center(2,1) + param.r(1)*sin(th);
    edge2x = param.center(1,2) + param.R*cos(th);
    edge2y = param.center(2,2) + param.R*sin(th);
    edge4x = param.center(1,2) + (param.R-sure)*cos(th);
    edge4y = param.center(2,2) + (param.R-sure)*sin(th);
    edge3x = param.center(1,3) + param.r(2)*cos(th);
    edge3y = param.center(2,3) + param.r(2)*sin(th);
    plot(param.center(1,1),param.center(2,1),'o');
    plot(param.center(1,2),param.center(2,2),'o');
    plot(param.center(1,3),param.center(2,3),'o');
    plot(edge1x,edge1y,'LineStyle','--','Color','b');
    plot(edge2x,edge2y,'LineStyle','--','Color',[0,0.5,1]);
    plot(edge4x,edge4y,'LineStyle','--','Color','k');
    plot(edge3x,edge3y,'LineStyle','--','Color','b');
end


quiver(param.point(1,1),param.point(2,1),15*cos(param.vtheta(1)),15*sin(param.vtheta(1)),'Color','r','LineWidth',1);
%    scatter(param.point(1,Le),param.point(2,Le),'*','r','LineWidth',1);
% line([param.point(1,Le),param.aim(1)],[param.point(2,Le),param.aim(2)],'LineStyle',':','LineWidth',1.5);


% scatter(param.aim(1),param.aim(2),'>','r','LineWidth',1);
text(param.point(1,1), param.point(2,1),['UAV']);
%    text(param.point(1,Le),param.point(2,Le),['UAV',num2str(i)]);
text(param.point(1,Le),param.point(2,Le),['TAR']);


title(['Length:',num2str(param.Length)])
end