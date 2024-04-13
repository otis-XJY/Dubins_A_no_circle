function [final_path,open,close]=A_dubins_nocircle_none(Start_Point,End_Point,obs,sure,r,obs_no_circle,obs_no_circle_in,outline_all,Stepsize,total_field)


tic



%A*算法开始
% figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%初始节点
open={};
close={};
open_f=[];

pos_id_mayday=[];

%%节点[点的位置(出发点)，方向(1个)，f,g，无path]
node.point=Start_Point(1:2)';
node.vtheta=Start_Point(3);
node.g=0;
node.f=node.g+obtain_h(node.point,End_Point(1:2));
node.pos_id=-1;%%%起点为-1,终点为-2
node.parent_id=-1;
close{end+1}=node;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%产生初始路径
% diary('123.txt');
[param_all_start{1},param_best_start{1}] = Dubins_no_obs(Start_Point, End_Point,r,r,Stepsize,-2,-1);
% for i=1:length(param_all_start{1})
%     subplot(2,2,i)
%     Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
%     Draw_path(param_all_start{1}(i),sure,1,3)
% 
% end
%     hold off
 [flag_all,obs_id_all]=obtain_flag_safe(1,param_all_start,obs_no_circle,outline_all);
 [min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(1,param_all_start,flag_all);
if ~isempty(path_safe_flag)
    disp('Find Goal!!');
%     figure
%     Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
    close=update_close(path_safe_flag(flag_min),param_all_start,End_Point,close);
    final_path=obtain_path(close);

else
    [obs_to_avoid, ~, ~] = unique(obs_id_all);

%%生成避障路线n(转弯半径只有1种)*4===>n=0,则选择最优直达
%%初始节点[点的位置(起点)，方向，f,g,path]
param_all={};
for i=1:length(obs_to_avoid)
    [param_all{i},param_best{i}] = Dubins_obs_nocircle(Start_Point,End_Point,outline_all,r,r,r,Stepsize,obs_to_avoid(i),-1,obs_no_circle,total_field);
%     [param_all{2,i},param_best{2,i}] = Dubins_obs(Start_Point,End_Point,obs(obs_to_avoid(i),1:2),r,r,obs(obs_to_avoid(i),3),Stepsize);
end

%%检查避障路线的from_path=1,to_path=2是否碰撞
% %（从当前节点到下一节点）
% hold off
% Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle);%绘制地图



 [flag_all,obs_id_all]=obtain_flag_safe(2,param_all,obs_no_circle,outline_all);
 [min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(1,param_all,flag_all);



%(路径数=需要躲避障碍物数*4)*障碍物数


%%选择不碰撞的路径，g=到起点的距离=path.1+path.2,h=直线距离？
%%节点[点的位置(2个=出发圆+障碍圆切入)，方向(1个)，f,g,path(2个=弧度+直线)]
% [open_f,open]=update_open(path_safe_flag,param_all,End_Point,open_f,open,close);

[open_f,open]=update_open(path_safe_flag,param_all,End_Point,open_f,open,close);


% param_safe=struct();
% param_safe.r=obtain_safeparam(path_safe_flag,param_all);
% param_safe.R={};
% safe_obs_num=max(length(param_safe.r),length(param_safe.R));
% 
% % display(safe_obs_num)
% % display(param_safe)
% 
% for i=1:safe_obs_num
%     [open_f,open]=check_in_open_new(param_safe,End_Point,open_f,open,close,i);
% end



%%选择最短节点

flag_min=find(open_f==min(open_f));

%%移除open，加入close
% open{flag_min}
close{end+1}=open{flag_min};
open(flag_min)=[];
open_f(flag_min)=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%循环开始
%%生成无碰撞路径2(半径不同)*4

while 1
param_all={};
[param_all{1},param_best{1}] = Dubins_no_obs([close{end}.point(:,end)',close{end}.vtheta(end)], End_Point,r,r,Stepsize,-2,close{end}.pos_id);
% [param_all{2},param_best{2}] = Dubins_no_obs([close{end}.point(:,end)',close{end}.vtheta(end)], End_Point,close{end}.R,r,Stepsize,-2,close{end}.pos_id);



%%判断所有的初始路径是否不碰撞(和自己碰撞直接排除)=>直接生成
% hold off
% Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle);%绘制地图

param_all=obtian_short_param(param_all);


 [flag_all,obs_id_all]=obtain_flag_safe(1,param_all,obs_no_circle,outline_all);
% %%如果有安全的
% %%选择安全中最短的
[min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(1,param_all,flag_all);


if ~isempty(path_safe_flag)
    disp('Find Goal!!');
%     figure

    close=update_close(path_safe_flag(flag_min),param_all,End_Point,close);
    final_path=obtain_path(close);

    break;
end


obs_num=obs_id_all;
[obs_to_avoid, ~, ~] = unique(obs_num);

rmove_id=[];
%%选出碰撞的障碍物(排除和自己的转弯圆碰撞)n个（及其路径）
for i=1:length(obs_to_avoid)
    if isequal(obs_to_avoid(i),close{end}.pos_id)
        rmove_id=[rmove_id,i];
    end
end
obs_to_avoid(rmove_id)=[];

% obs_num=obs_id_all;
% [obs_to_avoid, ~, ~] = unique(obs_num);


%%%%%%%%%%%%%%%%%%确定具体的躲避障碍

%%生成避障路线n(转弯半径2种)*4===>n=0,则选择最优直达
%%初始节点[点的位置(起点)，方向，f,g,path]
param_all={};
for i=1:length(obs_to_avoid)
    [param_all{1,i},param_best{1,i}] = Dubins_obs_nocircle([close{end}.point(:,end)',close{end}.vtheta],End_Point,outline_all,r,r,r,Stepsize,obs_to_avoid(i),close{end}.pos_id,obs_no_circle,total_field);
%      [param_all{2,i},param_best{2,i}] = Dubins_obs_nocircle([close{end}.point(:,end)',close{end}.vtheta],End_Point,outline_all,close{end}.R,r,sure,Stepsize,obs_to_avoid(i),close{end}.pos_id,obs_no_circle);

end



%检查1-2的安全
% hold off
% Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle);%绘制地图
 [flag_all,obs_id_all]=obtain_flag_safe(2,param_all,obs_no_circle,outline_all);
%  [flag_all,obs_id_all]=obtain_flag_safe(2,param_all,obs,Start_Point,End_Point,obs_know);

%%如果有安全的

[min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(2,param_all,flag_all);




%%选择不碰撞的路径，g=到起点的距离=path.1+path.2,h=直线距离？
%%节点[点的位置(2个=出发圆+障碍圆切入)，方向(1个)，f,g,path(2个=弧度+直线)]

% [open_f,open]=update_open(path_safe_flag.r,param_all(1,:),End_Point,open_f,open,close);
% [open_f,open]=update_open(path_safe_flag.R,param_all(2,:),End_Point,open_f,open,close);

% [open_f,open]=check_in_open(path_safe_flag.r,param_all(1,:),End_Point,open_f,open,close);
[open_f,open]=update_open(path_safe_flag,param_all,End_Point,open_f,open,close);



flag_min=find(open_f==min(open_f));

%%移除open，加入close
% open{flag_min}
close{end+1}=open{flag_min};
open(flag_min)=[];
open_f(flag_min)=[];

% display(close)



end

% Draw_pathA(close,sure)

end
% end
%%%%%%%%%%%%%%%%%循环结束
toc
disp(['运行时间: ',num2str(toc)]);



% % Draw_path(param_best{1},sure,1,2)
% % hold off
% figure
% Draw_map(Start_Point,End_Point,obs,sure,obs_no_circle,obs_no_circle_in);%绘制地图
% Draw_pathA_no_circle(close);
% hold off


end