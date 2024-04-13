function [path_apf,path_apf_all,X_APF_final_id]=APF_old(vtheta_all,X_end_A_dubins,angle,X_now_APF,X_APF_final,step_param,obs_param,apf_param,A_dubins_param,path_apf_all,path_apf,X_APF_final_id,i)

angle_CCW_CW=[];

fov=apf_param(1);
k_att=apf_param(2);
k_rep=apf_param(3);
K=apf_param(4);
r_search=apf_param(5);

obs_all=obs_param{1};
obs_unknow=obs_param{2};
obs_no_circle=obs_param{3};
obs_no_circle_in=obs_param{4};
num_obs=obs_param{5};
obs=obs_param{6};
obs_no_circle_unknow=obs_param{7};
obs_no_circle_in_unknow=obs_param{8};


Stepsize_apf=step_param(1);
Stepsize=step_param(2);
step=step_param(3);

Start_Point=A_dubins_param{1};
End_Point=A_dubins_param{2};
sure=A_dubins_param{3};
r=A_dubins_param{4};
draw_path=A_dubins_param{5};



for k=1:K %循环开始
    hold off
    






%     obs(:,1)=obs(:,1)+0.001*length(obs(:,1));
    path_apf(k,1)=X_now_APF(1); %path是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
    path_apf(k,2)=X_now_APF(2);
    %目标点对路径点的引力
    [Fatt_x,Fatt_y,att_angle,r_goal]=att_old(X_now_APF,X_APF_final,k_att,4);
    Fatt=sqrt(Fatt_x^2+Fatt_y^2);

    Frep_x_=0;Frep_y_=0;
    check_in=[];
    for i1=1:length(obs_all)
        check_in=[check_in,pointInSector(obs_all(i1,1),obs_all(i1,2),X_now_APF(1),X_now_APF(2),r_search,angle,fov)];
    end
    obs_range=find(check_in==1);
    %%%%斥力计算
    if ~isempty(obs_range)
        Frep_x=[];Frep_y=[];dis_obs_unknow=[];angle1=[];angle2=[];
        for i2=1:length(obs_range)
            [Frep_x(i2),Frep_y(i2),rep_angle,dis_obs_unknow(i2),angle1(i2),angle2(i2),Frep1(i2),Frep2(i2)]=rep_old(X_now_APF,obs_all(obs_range(i2),:),k_rep,r_goal,r_search);
        end
        
        min_r_obs=find(dis_obs_unknow==min(dis_obs_unknow));
        Frep_allx=sum(Frep_x);
        Frep_ally=sum(Frep_y);
        Frep_all=sqrt(Frep_allx^2+Frep_ally^2);
        Frep_x_=Frep_x(min_r_obs)*Frep_allx/Frep_all;
        Frep_y_=Frep_y(min_r_obs)*Frep_ally/Frep_all;
        Frep=sqrt(Frep_x_(1)^2+Frep_y_(1)^2);
        quiver(X_now_APF(1),X_now_APF(2),10*cos(rep_angle),10*sin(rep_angle),'r');hold on;
        quiver(obs_all(obs_range(min_r_obs(1)),1),obs_all(obs_range(min_r_obs(1)),2),100*cos(angle1(min_r_obs(1))),100*sin(angle1(min_r_obs(1))),'r');hold on;
        quiver(obs_all(obs_range(min_r_obs(1)),1),obs_all(obs_range(min_r_obs(1)),2),100*cos(angle2(min_r_obs(1))),100*sin(angle2(min_r_obs(1))),'r');hold on;
        text(-50,80, ['Frep：',num2str(Frep)]);
        text(-50,70, ['Frep1rep：',num2str(Frep1(min_r_obs(1)))]);
        text(-50,60, ['Frep2att：',num2str(Frep2(min_r_obs(1)))]);
    end


    quiver(X_now_APF(1),X_now_APF(2),100*cos(att_angle),100*sin(att_angle),'b');hold on;


    quiver(300,300,50*cos(0),50*sin(0),'r');hold on;
    text(300+50*cos(0),300+50*sin(0), 'Frep_{all}');
    
    quiver(300,290,50*cos(0),50*sin(0),'--r');hold on;
    text(300+50*cos(0),290+50*sin(0), 'Frep_{rep}');
    
    quiver(300,280,50*cos(0),50*sin(0),'-.r');hold on;
    text(300+50*cos(0),280+50*sin(0), 'Frep_{att}');
    
    quiver(300,270,50*cos(0),50*sin(0),'b');hold on;
    text(300+50*cos(0),270+50*sin(0), 'Fatt');
    
    quiver(300,260,50*cos(0),50*sin(0),'ok');hold on;
    text(300+50*cos(0),260+50*sin(0), 'v_{now}');
    
    quiver(300,250,50*cos(0),50*sin(0),'or');hold on;
    text(300+50*cos(0),250+50*sin(0), 'v_{next}');



    %计算合力
    Fsum_x=Fatt_x+Frep_x_(1);
    Fsum_y=Fatt_y+Frep_y_(1);
    F=sqrt(Fsum_x^2+Fsum_y^2);



    Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图


    draw_all(X_end_A_dubins,X_APF_final,X_now_APF,path_apf,angle,obs_no_circle_unknow,obs_no_circle_in_unknow,F,Fatt)
    draw_sector(angle,fov,X_now_APF,r_search,obs_range,obs_all)
    

    %%%%%%%%%%%动力学约束
    angle_new=atan2(Fsum_y,Fsum_x);
    quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'or');hold on;            
    [angle_new,angle_CCW_CW]=constrain_angle(angle,angle_new,angle_CCW_CW,Stepsize_apf,r);



        %求解下一个路径点
    Xnext(1)=(X_now_APF(1)+Stepsize_apf*cos(angle_new));   %式子中的l是步长
    Xnext(2)=(X_now_APF(2)+Stepsize_apf*sin(angle_new));
    X_now_APF=Xnext;

    angle=angle_new;



    if (sqrt((X_now_APF(1)-X_APF_final(1))^2+(X_now_APF(2)-X_APF_final(2))^2)<Stepsize_apf)   %当物体接近目标点时
        if abs(angle-vtheta_all(X_APF_final_id+1))>Stepsize_apf/r
            text(-10,-10, 'apf to dubins need angle_change','Interpreter', 'none');pause(3)
            display('apf to dubins need angle_change')
            [X_APF_final,X_APF_final_id]=obtain_apf_final(i+k*1/step+10/Stepsize,1/step,draw_path,obs_all,obs_range,obs_no_circle,2);
        else

    %         k=j;   %迭代次数
            text(-10,-10, 'find Goal!');pause(3)
            display('find Goal!');
            path_apf(k+1,1)=X_now_APF(1); %path是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
            path_apf(k+1,2)=X_now_APF(2);
            path_apf(k+2,1)=X_APF_final(1); %Goal是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
            path_apf(k+2,2)=X_APF_final(2);
    
            Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
            draw_all(X_end_A_dubins,X_APF_final,X_now_APF,path_apf,angle,obs_no_circle_unknow,obs_no_circle_in_unknow,F,Fatt)
            draw_sector(angle,fov,X_now_APF,r_search,obs_range,obs_all)
            path_apf_all=[path_apf_all;path_apf];
    
    %         drawnow
            break;
        end
    end


    safe_final=safe_apf_final(X_now_APF,X_APF_final,obs_all,obs_range,obs_no_circle,nan);

    if ~safe_final
        r_max=ceil(estimate_obs_unknow_r(obs_range,obs_all,X_now_APF));
%                     r_max=0;
        [X_APF_final,X_APF_final_id]=obtain_apf_final(i+k*1/step+(r_search+5*r_max)/Stepsize,2/step,draw_path,obs_all,obs_range,obs_no_circle,1);
        display('change X_APF_final')
        text(-10,-10, 'change X_ APF_final', 'Interpreter', 'none');pause(3)
    end

    plot(X_APF_final(1),X_APF_final(2),'x')
    
    pause(0.001)
end

end