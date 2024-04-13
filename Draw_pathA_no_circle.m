function  Draw_pathA_no_circle(close)
num=length(close);

axis equal;hold on
for i=2:num-1
    for j=2:length(close{i}.point(1,:))
        plot(close{i}.point(1,j),close{i}.point(2,j),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
        quiver(close{i}.point(1,j),close{i}.point(2,j),15*cos(close{i}.vtheta_all{j-1}(end)),15*sin(close{i}.vtheta_all{j-1}(end)),'Color','r','LineWidth',1);
    end

    for le=1:length(close{i}.path)
        plot(close{i}.path{le}(1,:),close{i}.path{le}(2,:),'LineWidth',1);
    end


    for k=1:2:length(close{i}.center(1,:))-1
        th = 0:0.1:2*pi;
        edge1x = close{i}.center(1,k) + close{i}.r(1)*cos(th);
        edge1y = close{i}.center(2,k) + close{i}.r(1)*sin(th);
        plot(close{i}.center(1,k),close{i}.center(2,k),'o');
        plot(edge1x,edge1y,'LineStyle','--','Color','r');
%         display([close{i}.center(1,k),close{i}.center(2,k)])


        if k+1<=length(close{i}.center(1,:))
            edge2x = close{i}.center(1,k+1) + close{i}.R*cos(th);
            edge2y = close{i}.center(2,k+1) + close{i}.R*sin(th);
%             edge4x = close{i}.center(1,k+1) + (close{i}.R-sure)*cos(th);
%             edge4y = close{i}.center(2,k+1) + (close{i}.R-sure)*sin(th);
            
            plot(close{i}.center(1,k+1),close{i}.center(2,k+1),'^');
    
            plot(edge2x,edge2y,'LineStyle','--','Color',[0,0.5,1],'Marker','*');
%             plot(edge4x,edge4y,'LineStyle','--','Color','y','Marker','*');
        end
    end

    end

% end


for j=2:length(close{num}.point(1,:))
        plot(close{num}.point(1,j),close{num}.point(2,j),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
end
    quiver(close{num}.point(1,2),close{num}.point(2,2),15*cos(close{num}.vtheta(1)),15*sin(close{num}.vtheta(1)),'Color','r','LineWidth',1);
    quiver(close{num}.point(1,3),close{num}.point(2,3),15*cos(close{num}.vtheta(1)),15*sin(close{num}.vtheta(1)),'Color','r','LineWidth',1);
    quiver(close{num}.point(1,4),close{num}.point(2,4),15*cos(close{num}.vtheta(2)),15*sin(close{num}.vtheta(2)),'Color','r','LineWidth',1);
    
    for le=1:length(close{num}.path)
        plot(close{num}.path{le}(1,:),close{num}.path{le}(2,:),'LineWidth',1);
    end
    th = 0:0.1:2*pi;
    edge1x = close{num}.center(1,1) + close{num}.r(1)*cos(th);
    edge1y = close{num}.center(2,1) + close{num}.r(1)*sin(th);
    edge2x = close{num}.center(1,2) + close{num}.r(2)*cos(th);
    edge2y = close{num}.center(2,2) + close{num}.r(2)*sin(th);
    plot(close{num}.center(1,1),close{num}.center(2,1),'o');
    plot(close{num}.center(1,2),close{num}.center(2,2),'o');
    plot(edge1x,edge1y,'LineStyle','--','Color','r');
    plot(edge2x,edge2y,'LineStyle','--','Color','b');
    
%     display([close{num}.center(1,1),close{num}.center(2,1)])
%     display([close{num}.center(1,2),close{num}.center(2,2)])


plot(close{1}.point(1,1),close{1}.point(2,1),'.','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');
quiver(close{1}.point(1,1),close{1}.point(2,1),15*cos(close{1}.vtheta),15*sin(close{1}.vtheta),'Color','r','LineWidth',1);

% final_length=0;
% for i=2:num
%     final_length=final_length+close{i}.Length;
% end

xlabel(['Length:',num2str(close{end}.alllength)])
% ylabel(['Length:',num2str(final_length)])
end