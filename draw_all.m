function draw_all(X_start,X_final,X_now,path,angle,obs_no_circle,obs_no_circle_in,F,Fatt)
    plot(X_final(1),X_final(2),'v');hold on;
    plot(X_start(1),X_start(2),'ms');hold on;
    plot(path(:,1),path(:,2),'.r','LineWidth',1,'LineStyle','--');hold on;

    quiver(X_now(1),X_now(2),100*cos(angle),100*sin(angle),'ok');hold on;
%     text(X_now(1)+100*cos(angle),X_now(2)+100*sin(angle), 'v_now');

    text(-50,100,['F：',num2str(F)]);
    text(-50,90,['Fatt：',num2str(Fatt)]);

    Draw_obs_unknow_circle(obs_no_circle,obs_no_circle_in)

end