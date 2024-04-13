function [path_apf,path_apf_all,X_APF_final_id]=APF0407(vtheta_all,X_end_A_dubins,angle,X_now_APF,X_APF_final,step_param,obs_param,apf_param,A_dubins_param,path_apf_all,path_apf,X_APF_final_id,i)

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

obs_finall=[];
Stepsize_apf=step_param(1);
Stepsize=step_param(2);
step=step_param(3);

Start_Point=A_dubins_param{1};
End_Point=A_dubins_param{2};
sure=A_dubins_param{3};
r=A_dubins_param{4};
draw_path=A_dubins_param{5};

flag_realdager=0;
flag_new=0;
 min_obs=[0,0,inf];
flag_danger=0;
unreacheable=0;
flag_danger_fuck=0;
empty_flag=0;
for k=1:K %循环开始
    hold off
    obs_finall=[];
    
    




%     obs(:,1)=obs(:,1)+0.001*length(obs(:,1));
    path_apf(k,1)=X_now_APF(1); %path是保存车走过的每个点的坐标。刚开始先将起点放进该向量。
    path_apf(k,2)=X_now_APF(2);
    %目标点对路径点的引力
    [Fatt_x,Fatt_y,att_angle,r_goal]=att(X_now_APF,X_APF_final,k_att,4);
    Fatt=sqrt(Fatt_x^2+Fatt_y^2);

    Frep_x=0;Frep_y=0;
    check_in=[];
    for i1=1:length(obs_all)
        check_in=[check_in,pointInSector(obs_all(i1,1),obs_all(i1,2),X_now_APF(1),X_now_APF(2),r_search,angle,fov)];
    end
    obs_range=find(check_in==1);%%%%探测感知视野内的障碍物的id=>obs_unknow(obs_range(j),1)为实际坐标
    if ~isempty(obs_range)
        angle_obs=atan2(obs_all(obs_range,2)-X_now_APF(2),obs_all(obs_range,1)-X_now_APF(1));
        distance_obs=sqrt((obs_all(obs_range,2)-X_now_APF(2)).^2+(obs_all(obs_range,1)-X_now_APF(1)).^2); 
    
        matirx=angle_obs-angle_obs';
        for ll=1:length(angle_obs)
            idll=find(abs(matirx(ll,:))<0.1);
            min_idll=find(distance_obs==min(distance_obs(idll)));
            obs_finall=[obs_finall,obs_range(min_idll)];
    
        end
        obs_range=unique(obs_finall);
    end



%     obs_range = pointInSector_cons(obs_all,obs_range,X_end_A_dubins);
    %%%%斥力计算
    if ~isempty(obs_range)
        Frep_x=[];Frep_y=[];dis_obs_unknow=[];angle1=[];angle2=[];Frep1=[];Frep2=[];
        for i2=1:length(obs_range)
            [Frep_x(i2),Frep_y(i2),rep_angle,dis_obs_unknow(i2),angle1(i2),angle2(i2),Frep1(i2,:),Frep2(i2,:)]=rep(X_now_APF,obs_all(obs_range(i2),:),k_rep,r_goal,r_search);
        end





        angle_obs=atan2(obs_all(obs_range,2)-X_now_APF(2),obs_all(obs_range,1)-X_now_APF(1));
        distance_obs=sqrt((obs_all(obs_range,2)-X_now_APF(2)).^2+(obs_all(obs_range,1)-X_now_APF(1)).^2); 
        angle_pre=nan;%%%一个不可能数
        if ~isempty(angle_obs)
    %         angle_obs
%                 cha=abs(angle_obs-angle_new);
            cha=abs(transform_theta(abs(transform_theta_2pi(angle_obs)-transform_theta_2pi(angle))));
%                 min(cha)
%             r_threta=inf;
            if ~isempty(find(cha<0.02))
%                 r_threta=[inf,distance_obs(find(cha<0.008))'];
                cha_pre=transform_theta_2pi(angle_obs)-transform_theta_2pi(angle);
                cha_pre(find(cha_pre>pi))=2*pi-cha_pre(find(cha_pre>pi));
                [value_max,id_max]=max(cha_pre);
                [value_min,id_min]=min(cha_pre);
                if distance_obs(id_min)>distance_obs(id_max)
                    id_pre=id_min;
                else
                    id_pre=id_max;
                end
                
                [Fpre_x,Fpre_y,angle_pre,r_pre]=pre(X_now_APF,obs_all(obs_range(id_pre),:),k_att,4);
                Fpre=sqrt(Fpre_x^2+Fpre_y^2);

                quiver(obs_all(obs_range(id_pre(1)),1),obs_all(obs_range(id_pre(1)),2),100*cos(angle_pre),100*sin(angle_pre),'-.k');hold on;
                text(-50,50, ['Fpre：',num2str(Fpre)]);

            end
%             r_threta=min(r_threta);
        end
    
        min_r_obs=find(dis_obs_unknow==min(dis_obs_unknow));
        max_r_obs=find(dis_obs_unknow==max(dis_obs_unknow));





        frep_repx=sum(Frep1(:,1));
        frep_repy=sum(Frep1(:,2));
        frep_rep_all=sqrt(frep_repx^2+frep_repy^2);
        Ferp_rep_x=abs(Frep1(min_r_obs,1))*frep_repx/frep_rep_all;
        Ferp_rep_y=abs(Frep1(min_r_obs,2))*frep_repy/frep_rep_all;
        Ferp_rep_all=sqrt(Ferp_rep_x(1)^2+Ferp_rep_y(1)^2);
        repangle=atan2(Ferp_rep_y(1),Ferp_rep_x(1));

        frep_attx=sum(Frep2(:,1));
        frep_atty=sum(Frep2(:,2));
        frep_att_all=sqrt(frep_attx^2+frep_atty^2);
        Ferp_att_x=abs(Frep2(max_r_obs,1))*frep_attx/frep_att_all;
        Ferp_att_y=abs(Frep2(max_r_obs,2))*frep_atty/frep_att_all;
        Ferp_att_all=sqrt(Ferp_att_x(1)^2+Ferp_att_y(1)^2);
        attangle=atan2(Ferp_att_y(1),Ferp_att_x(1));


        Frep_x=Ferp_att_x(1)+Ferp_rep_x(1);
        Frep_y=Ferp_att_y(1)+Ferp_rep_y(1);
        Frep_all=sqrt(Frep_x(1)^2+Frep_y(1)^2);
        rep_angle=atan2(Frep_y(1),Frep_x(1));
        
%         Frep_allx=sum(Frep_x);
%         Frep_ally=sum(Frep_y);
%         Frep_all=sqrt(Frep_allx^2+Frep_ally^2);
%         Frep_x_=Frep_x(min_r_obs)*Frep_allx/Frep_all;
%         Frep_y_=Frep_y(min_r_obs)*Frep_ally/Frep_all;
%         rep_angle=atan2(Frep_y_(1),Frep_x_(1));
%         Frep=sqrt(Frep_x_(1)^2+Frep_y_(1)^2);




        quiver(X_now_APF(1),X_now_APF(2),100*cos(rep_angle),100*sin(rep_angle),'r');hold on;
        quiver(obs_all(obs_range(min_r_obs(1)),1),obs_all(obs_range(min_r_obs(1)),2),100*cos(repangle),100*sin(repangle),'--r');hold on;
        quiver(obs_all(obs_range(max_r_obs(1)),1),obs_all(obs_range(max_r_obs(1)),2),100*cos(attangle),100*sin(attangle),'-.r');hold on;
        text(-50,80, ['Frep：',num2str(Frep_all)]);
        text(-50,70, ['Frep1rep：',num2str(Ferp_rep_all)]);
        text(-50,60, ['Frep2att：',num2str(Ferp_att_all)]);
        

        if  min(dis_obs_unknow)<min_obs(3) && flag_realdager==0
            text(350,-10,'change thereate');
            flag_danger=0;
            
        end


        min_obs=[obs_all(obs_range(min_r_obs(1)),:),min(dis_obs_unknow),angle1(min_r_obs(1))];
        max_obs=[obs_all(obs_range(max_r_obs(1)),:),max(dis_obs_unknow),angle1(max_r_obs(1))];
        empty_flag=0;
        flag_new=1;
    else
        min_obs=[0,0,inf];
        empty_flag=empty_flag+1;
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

    text(350,50,num2str(k))
    if k==80
        display(k)
    end



    %计算合力
    Fsum_x=Fatt_x+Frep_x(1);
    Fsum_y=Fatt_y+Frep_y(1);
    F=sqrt(Fsum_x^2+Fsum_y^2);



    Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图


    draw_all(X_end_A_dubins,X_APF_final,X_now_APF,path_apf,angle,obs_no_circle_unknow,obs_no_circle_in_unknow,F,Fatt)
    draw_sector(angle,fov,X_now_APF,r_search,obs_range,obs_all)
    

    %%%%%%%%%%%动力学约束
    angle_new=atan2(Fsum_y,Fsum_x);
    quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'or');hold on; 


    if min_obs(1)~=0
        angle_therate=atan2(min_obs(2)-X_now_APF(2),min_obs(1)-X_now_APF(1));
        angle_pre2=atan2(max_obs(2)-X_now_APF(2),max_obs(1)-X_now_APF(1));
        text(350,10,[num2str(min_obs(3)),'fuck',num2str(abs(transform_theta(abs(transform_theta_2pi(angle_therate)-transform_theta_2pi(angle_new)))))])
%         if min_obs(3)<1.5*r && min_obs(3)>1.1*r  || abs(transform_theta(abs(transform_theta_2pi(angle_therate)-transform_theta_2pi(angle_new))))<0.2%%0.7330
        if min_obs(3)<1.5*r || abs(transform_theta(abs(transform_theta_2pi(angle_therate)-transform_theta_2pi(angle_new))))<0.2%%0.7330
            %%
            if flag_danger==0
                flag_danger=1;
            else
                flag_danger=flag_danger+1;
            end

%             display(flag_danger)
            text(350,0,['dager',num2str(flag_danger)]);
            text(-10,-30,'dager');
%                 pause(1);
            angle_ture1=angle+Stepsize_apf/r;
            angle_ture2=angle-Stepsize_apf/r;
            angle_ture1=transform_theta(mod(angle_ture1,2*pi));
            angle_ture2=transform_theta(mod(angle_ture2,2*pi));
            quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_ture1),100*sin(angle_ture1),'og');hold on;  
            quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_ture2),100*sin(angle_ture2),'om');hold on; 



            %%
            if flag_danger==1
                if abs(transform_theta(abs(transform_theta_2pi(repangle)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs(transform_theta_2pi(repangle) ...
                        -transform_theta_2pi(angle_ture2))))
                    angle_new=angle_ture1;
                    angle_fov=angle+Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                    angle_end=angle-0.8*Stepsize_apf/r;
                    flag_danger_angle=1;
                else
                    angle_new=angle_ture2;
                    angle_fov=angle-Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                    angle_end=angle+0.8*Stepsize_apf/r;
                    flag_danger_angle=2;
                end

            else
                if flag_danger_angle==1
                    angle_new=angle_ture1;
                    angle_fov=angle+Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                    angle_end=angle-0.8*Stepsize_apf/r;
                else
                    angle_new=angle_ture2;
                    angle_fov=angle-Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                    angle_end=angle+0.8*Stepsize_apf/r;
                end

            end
            %%
            angle_end=atan2(sin(attangle)+2*sin(angle_end),cos(attangle)+2*cos(angle_end));
            X_APF_final(1)=(X_now_APF(1)+5*Stepsize_apf*cos(angle_pre2));   %式子中的l是步长
            X_APF_final(2)=(X_now_APF(2)+5*Stepsize_apf*sin(angle_pre2));
            plot(X_APF_final(1),X_APF_final(2),'ro')

%                     quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'--c');hold on;
            if ~isnan(angle_pre)
                text(-10,-40,'realdager');

                %%
               if abs(transform_theta(abs(transform_theta_2pi(angle_pre)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs(transform_theta_2pi(angle_pre) ...
                        -transform_theta_2pi(angle_ture2))))
                    angle_new=angle_ture1;
                    angle_fov=angle+Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                else
                    angle_new=angle_ture2;
                    angle_fov=angle-Stepsize_apf/r/2;
                    angle_fov=transform_theta(mod(angle_fov,2*pi));
                end
 
    

                angle_end=atan2(sin(angle_pre)+2*sin(angle_new),cos(angle_pre)+2*cos(angle_new));
                [vlaue,id_end]=max([abs(angle_fov-angle),abs(angle_pre-angle)]);
                angle_end=[angle_fov,angle_pre];
                X_APF_final(1)=(X_now_APF(1)+5*Stepsize_apf*cos(angle_pre));   %式子中的l是步长
                X_APF_final(2)=(X_now_APF(2)+5*Stepsize_apf*sin(angle_pre));
                plot(X_APF_final(1),X_APF_final(2),'ro')

%                 quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_end),'--c');hold on;

            end


% % %         else
% % %%
% %             X_predeict(1)=(X_now_APF(1)+Stepsize_apf*cos(angle_new));   %式子中的l是步长
% %             X_predeict(2)=(X_now_APF(2)+Stepsize_apf*sin(angle_new));
% %             check_in=[];
% %             for i1=1:length(obs_all)
% %                 check_in=[check_in,pointInSector(obs_all(i1,1),obs_all(i1,2),X_predeict(1),X_predeict(2),r_search,angle,fov)];
% %             end
% %             obs_range_pre=find(check_in==1);%%%%探测感知视野内的障碍物的id=>obs_unknow(obs_range(j),1)为实际坐标
% %             if ~isempty(obs_range_pre)
% %                 angle_obs_pre=atan2(obs_all(obs_range_pre,2)-X_predeict(2),obs_all(obs_range_pre,1)-X_predeict(1));
% %                 distance_obs=sqrt((obs_all(obs_range_pre,2)-X_predeict(2)).^2+(obs_all(obs_range_pre,1)-X_predeict(1)).^2); 
% %             
% %                 matirx=angle_obs_pre-angle_obs_pre';
% %                 for ll=1:length(angle_obs_pre)
% %                     idll=find(abs(matirx(ll,:))<0.008);
% %                     min_idll=find(distance_obs==min(distance_obs(idll)));
% %                     obs_finall=[obs_finall,obs_range_pre(min_idll)];
% %             
% %                 end
% %                 obs_range_pre=unique(obs_finall);
% %             end
% % 
% % 
% %             angle_obs=atan2(obs_all(obs_range_pre,2)-X_predeict(2),obs_all(obs_range_pre,1)-X_predeict(1));
% %             distance_obs=sqrt((obs_all(obs_range_pre,2)-X_predeict(2)).^2+(obs_all(obs_range_pre,1)-X_predeict(1)).^2); 
% %             if ~isempty(angle_obs)
% %         %         angle_obs
% % %                 cha=abs(angle_obs-angle_new);
% %                 cha=abs(transform_theta(abs(transform_theta_2pi(angle_obs)-transform_theta_2pi(angle_new))));
% % %                 min(cha)
% %                 r_threta=inf;
% %                 if ~isempty(find(cha<0.008))
% %                     r_threta=[inf,distance_obs(find(cha<0.008))'];
% %                 end
% %                 r_threta=min(r_threta);
% %             end
% % 
% %             %%
% %             if r_threta<1.1*r ||flag_realdager==1
% %                 %%
% %                 
% %     %             display(flag_realdager)
% %                 if flag_danger_fuck==0
% %                     flag_danger_fuck=1;
% %                 else
% %                     flag_danger_fuck=flag_danger_fuck+1;
% %                 end
% %     
% %     %             display('danger')
% %     %             display(flag_danger)
% %                 text(350,0,['realdager',num2str(flag_danger_fuck)]);
% %                 text(-10,-20,'realdager');
% %     %                 pause(1);
% %                 angle_ture1=angle+Stepsize_apf/r;
% %                 angle_ture2=angle-Stepsize_apf/r;
% %                 angle_ture1=transform_theta(mod(angle_ture1,2*pi));
% %                 angle_ture2=transform_theta(mod(angle_ture2,2*pi));
% %                 quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_ture1),100*sin(angle_ture1),'og');hold on;  
% %                 quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_ture2),100*sin(angle_ture2),'om');hold on; 
% % 
% % 
% %                 X_APF_final(1)=(X_now_APF(1)+1*Stepsize_apf*cos(angle));   %式子中的l是步长
% %                 X_APF_final(2)=(X_now_APF(2)+1*Stepsize_apf*sin(angle));
% %                 plot(X_APF_final(1),X_APF_final(2),'ro')
% % 
% % %%
% %                 if flag_danger_fuck==1
% %                     if abs(transform_theta(abs(transform_theta_2pi(repangle)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs(transform_theta_2pi(repangle) ...
% %                             -transform_theta_2pi(angle_ture2))))
% %                         angle_new=angle_ture1;
% %                         flag_danger_angle=1;
% %                     else
% %                         angle_new=angle_ture2;
% %                         flag_danger_angle=2;
% %                     end
% % 
% %                 else
% %                     if flag_danger_angle==1
% %                         angle_new=angle_ture1;
% %                     else
% %                         angle_new=angle_ture2;
% %                     end
% %     
% %                 end
% % %%
% % 
% % 
% % %                     quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'--c');hold on;
% % 
% %                 if ~isnan(angle_pre)
% %                     if abs(transform_theta(abs(transform_theta_2pi(angle_pre)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs(transform_theta_2pi(angle_pre) ...
% %                             -transform_theta_2pi(angle_ture2))))
% %                         angle_new=angle_ture1;
% %                         flag_danger_angle=1;
% %                     else
% %                         angle_new=angle_ture2;
% %                         flag_danger_angle=2;
% %                     end
% %                         angle_end=antan2(sin(angle_pre)+2*sin(angle_new),cos(angle_pre)+2*cos(angle_new));
% %                         X_APF_final(1)=(X_now_APF(1)+1*Stepsize_apf*cos(angle_end));   %式子中的l是步长
% %                         X_APF_final(2)=(X_now_APF(2)+1*Stepsize_apf*sin(angle_end));
% %                         plot(X_APF_final(1),X_APF_final(2),'ro')
% % 
% %                         quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_end),100*sin(angle_end),'--c');hold on;
% % 
% %                 end
% % 
% %                 if flag_realdager==0
% %                     angle_danger=angle;
% %                     flag_realdager=1;
% %     
% %                 end
% %     %%
% %                 display([angle_danger,angle_new,flag_danger,flag_danger_fuck])
% %     
% %     
% %     
% % %                 if abs(transform_theta(abs(transform_theta_2pi(angle_danger)-transform_theta_2pi(angle_new))))>pi/2 
% % %                     flag_realdager=0;
% % %                     flag_danger_fuck=0;
% % %                 end
% %             else
% %                 flag_danger_fuck=0;
% %                 flag_realdager=0;
% %             end
    
        else
            flag_danger=0;
            flag_danger_fuck=0;
            flag_realdager=0;
            [angle_fov,angle_new,angle_CCW_CW]=constrain_angle(angle,angle_new,angle_CCW_CW,Stepsize_apf,r);
            quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'ob');hold on;  
        end
    else
        flag_danger=0;
        flag_danger_fuck=0;
        flag_realdager=0;
        [angle_fov,angle_new,angle_CCW_CW]=constrain_angle(angle,angle_new,angle_CCW_CW,Stepsize_apf,r);
        quiver(X_now_APF(1),X_now_APF(2),100*cos(angle_new),100*sin(angle_new),'ob');hold on;  
    end
    
    dis_to_end=sqrt((X_now_APF(1)-X_APF_final(1))^2+(X_now_APF(2)-X_APF_final(2))^2); 
    X_APF_final_id_dis=sqrt((draw_path(2,:)-X_now_APF(2)).^2+(draw_path(1,:)-X_now_APF(1)).^2); 
    % 找到数组的中位数
    [sorted_array, sorted_index] = sort(X_APF_final_id_dis);
    min_values = sorted_array(ceil(length(X_APF_final_id_dis)/2):ceil(length(X_APF_final_id_dis)/2)+1);
%     X_APF_final_id_obtain = sorted_index(length(X_APF_final_id_dis)/2:length(X_APF_final_id_dis)/2+1);
%     X_APF_final_id_obtain=max(X_APF_final_id_obtain);

    X_APF_final_id = sorted_index(1:2);
    X_APF_final_id=max(X_APF_final_id);


    X_APF_final_id_dis=sqrt((draw_path(2,X_APF_final_id:end)-X_now_APF(2)).^2+(draw_path(1,X_APF_final_id:end)-X_now_APF(1)).^2); 


    X_APF_final_id_obtain = sorted_index(ceil(length(X_APF_final_id_dis)/2):ceil(length(X_APF_final_id_dis)/2)+1);
    X_APF_final_id_obtain=max(X_APF_final_id_obtain);


    minX_APF_final_id = sorted_index(ceil(length(X_APF_final_id_dis)/15):ceil(length(X_APF_final_id_dis)/15)+1);
    minX_APF_final_id=max(minX_APF_final_id);
%     X_APF_final_id=sorted_index(1);

    
    % 找到中位数的索引
%     X_APF_final_id = find(sorted_array == median_value);
%     X_APF_final_id=X_APF_final_id+100;

    





        %求解下一个路径点
    Xnext(1)=(X_now_APF(1)+Stepsize_apf*cos(angle_new));   %式子中的l是步长
    Xnext(2)=(X_now_APF(2)+Stepsize_apf*sin(angle_new));
    X_now_APF=Xnext;

    angle=angle_new;

    
    if empty_flag>1 && flag_new==1 
            text(-10,-10, 'APF global new plan');
%             pause(3)
            display('APF global new plan')
            [X_APF_final,X_APF_final_id]=obtain_apf_final(X_APF_final_id_obtain,1,draw_path,obs_all,obs_range,obs_no_circle,2);
            empty_flag=0;
            flag_new=0;
    end



    if sqrt((X_now_APF(1)-X_APF_final(1))^2+(X_now_APF(2)-X_APF_final(2))^2)>dis_to_end%-0.5*Stepsize_apf
        %%
        unreacheable=unreacheable+1;
    
   
            text(-10,-10, 'unreacheable','Interpreter', 'none');
%             pause(3)
            display('unreacheable')
            [X_APF_final,X_APF_final_id]=obtain_apf_final(X_APF_final_id_obtain+ceil(unreacheable^(1.2))/Stepsize,1/step,draw_path,obs_all,obs_range,obs_no_circle,1);
    else
        unreacheable=0;
    end
    

    if (sqrt((X_now_APF(1)-X_APF_final(1))^2+(X_now_APF(2)-X_APF_final(2))^2)<Stepsize_apf)   %当物体接近目标点时
        %%
        if sorted_array(1)>Stepsize_apf
            text(-10,-10, 'APF global');
%             pause(3)
            display('APF global')
            [X_APF_final,X_APF_final_id]=obtain_apf_final(X_APF_final_id_obtain+1,1,draw_path,obs_all,obs_range,obs_no_circle,2);
        else
            if abs(transform_theta(abs(transform_theta_2pi(angle)-transform_theta_2pi(vtheta_all(X_APF_final_id+1)))))>Stepsize_apf/r
                text(-10,-10, 'apf to dubins need angle_change','Interpreter', 'none');
    %             pause(3)
                display('apf to dubins need angle_change')
                [X_APF_final,X_APF_final_id]=obtain_apf_final(minX_APF_final_id+1,1,draw_path,obs_all,obs_range,obs_no_circle,2);
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
    end


    safe_final=safe_apf_final(X_now_APF,X_APF_final,obs_all,obs_range,obs_no_circle,min(dis_obs_unknow));
    if ~safe_final
        %%
        r_max=ceil(estimate_obs_unknow_r(obs_range,obs_all,X_now_APF));
%                     r_max=0;
        [X_APF_final,X_APF_final_id]=obtain_apf_final(X_APF_final_id_obtain+(r_search+5*r_max)/Stepsize,2/step,draw_path,obs_all,obs_range,obs_no_circle,1);
        display('change X_APF_final')
        text(-10,-10, 'change X_ APF_final', 'Interpreter', 'none');
%         pause(3)
    end

    plot(X_APF_final(1),X_APF_final(2),'rx')
    
    pause(0.001)
end

end