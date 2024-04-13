function [Frep_x,Frep_y,angle,r_obs,angle1,angle2,Frep1,Frep2]=rep_old(pos,obs,k_rep,r_goal,safe_obs)
    deltaX_obs=obs(1)-pos(1);
    deltaY_obs=obs(2)-pos(2);
    r_obs=sqrt(deltaX_obs^2+deltaY_obs^2);  %路径点和目标的距离
    Frep_x2=0;
    Frep_y2=0;

    %目标点对路径点的引力
    Frep_x1=-k_rep*(1/r_obs-1/safe_obs)/(r_obs^2)*r_goal(3)*(deltaX_obs/r_obs);
    Frep_y1=-k_rep*(1/r_obs-1/safe_obs)/(r_obs^2)*r_goal(3)*(deltaY_obs/r_obs);
    Frep_x2=k_rep/2*(1/r_obs-1/safe_obs)^2*r_goal(3)*(r_goal(1)/r_goal(3));
    Frep_y2=k_rep/2*(1/r_obs-1/safe_obs)^2*r_goal(3)*(r_goal(2)/r_goal(3));
    
    Frep_x1=-k_rep*(1/r_obs-1/safe_obs)^2/(r_obs^2)*(deltaX_obs/r_obs);
    Frep_y1=-k_rep*(1/r_obs-1/safe_obs)^2/(r_obs^2)*(deltaY_obs/r_obs);

    Frep_x2=0;
    Frep_y2=0;

    angle1=atan2(Frep_y1,Frep_x1);
    angle2=atan2(Frep_y2,Frep_x2);
    Frep1=sqrt(Frep_x1^2+Frep_y1^2);
    Frep2=sqrt(Frep_x2^2+Frep_y2^2);
%     display([Frep1;Frep2])


    Frep_x=Frep_x1+Frep_x2;
    Frep_y=Frep_y1+Frep_y2;
    angle=atan2(Frep_y,Frep_x);
end