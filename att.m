function [Fatt_x,Fatt_y,angle,r_goal]=att(pos,goal,k_att,obs_num)
    deltaX_goal=goal(1)-pos(1);
    deltaY_goal=goal(2)-pos(2);
    r_final=sqrt(deltaX_goal^2+deltaY_goal^2);  %路径点和目标的距离
    r_goal=[deltaX_goal,deltaY_goal,r_final];

    %目标点对路径点的引力
    Fatt_x=k_att*(r_final)^(1/obs_num)*(deltaX_goal/r_final);
    Fatt_y=k_att*(r_final)^(1/obs_num)*(deltaY_goal/r_final);
    angle=atan2(Fatt_y,Fatt_x);
end