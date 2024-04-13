function [param_all,param_best] = Dubins_no_obs(Start_Point,End_Point,r_s,r_e,Stepsize)
param_best.point = [];%切点集合
param_best.length = [];%每个路段长度
param_best.phy = [];%每个弧段的角度
param_best.center = [];%圆心
param_best.theta = [];
param_best.r = [r_s,r_e];
% param_best.R = R;
param_best.type = -1;%1-->RSR;2-->RSL;3-->LSL;4-->LSR 


% param_all.point = [];%切点集合
% param_all.length = [];%每个路段长度
% param_all.phy = [];%每个弧段的角度
% param_all.center = [];%圆心
% param_all.theta = [];
% param_all.r = [];
% % param_all.R = R;
% param_all.type = [];%1-->RSR;2-->RSL;3-->LSL;4-->LSR 
% param_all.vtheta = [];
% param_all.Length = [];



param1=struct();
param2=struct();
param3=struct();
param4=struct();

LengthfsRSR=inf;
LengthfsRSL=inf;
LengthfsLSL=inf;
LengthfsLSR=inf;



Start_X = Start_Point(1);
Start_Y = Start_Point(2);
Start_Theta = Start_Point(3);

End_X = End_Point(1);
End_Y = End_Point(2);
End_Theta = End_Point(3);

% Dx = Avoid_Center(1);
% Dy = Avoid_Center(2);



%% 计算初始圆心
CenterRs_x = Start_X + r_s*cos(Start_Theta - pi/2);
CenterRs_y = Start_Y + r_s*sin(Start_Theta - pi/2);       %右转圆
CenterLs_x = Start_X + r_s*cos(Start_Theta + pi/2);
CenterLs_y = Start_Y + r_s*sin(Start_Theta + pi/2);       %左转圆

%% 计算末位置圆心
CenterRf_x = End_X + r_e*cos(End_Theta - pi/2);   
CenterRf_y = End_Y + r_e*sin(End_Theta - pi/2);           %右转圆
CenterLf_x = End_X + r_e*cos(End_Theta + pi/2);
CenterLf_y = End_Y + r_e*sin(End_Theta + pi/2);           %左转圆
% %% 终点圆与目标区域的切点（右转圆） 
% cR = sqrt((CenterRf_x - Dx)^2 + (CenterRf_y - Dy)^2);
% alphaSR_out = asin((r_e-R)/cR);
% beltSR = atan2((CenterRf_y - Dy ),(CenterRf_x - Dx  ));
% 
% Theta_SRG0 = beltSR + alphaSR_out + pi/2 ;
% Theta_SRF0 = beltSR + alphaSR_out + pi/2 ;
% Theta_SRG = mod(Theta_SRG0 , 2*pi);
% Theta_SRF = mod(Theta_SRF0 , 2*pi);
% 
% x_SRG = Dx + R*cos(Theta_SRF) ;
% y_SRG = Dy + R*sin(Theta_SRF) ;%障碍圆上的点
% x_SRF = CenterRf_x + r_e*cos(Theta_SRG) ;
% y_SRF = CenterRf_y + r_e*sin(Theta_SRG) ;%终点圆上的点
% 
% % plot(x_SRG,y_SRG,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');
% % plot(x_SRF,y_SRF,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');
% 
% %% 终点圆与目标区域的切点（左转圆） 
% cL = sqrt((CenterLf_x - Dx)^2 + (CenterLf_y - Dy)^2);
% alphaSL_out = asin((r_e-R)/cL);
% beltSL = atan2((CenterLf_y - Dy ),( CenterLf_x - Dx ));
% 
% Theta_SLG0 = beltSL - alphaSL_out + 3*pi/2 ;
% Theta_SLF0 = beltSL - alphaSL_out + 3*pi/2 ;
% Theta_SLG = mod(Theta_SLG0 , 2*pi);
% Theta_SLF = mod(Theta_SLF0 , 2*pi);
% 
% x_SLG = Dx + R*cos(Theta_SLF) ;
% y_SLG = Dy + R*sin(Theta_SLF) ;
% x_SLF = CenterLf_x + r_e*cos(Theta_SLG) ;
% y_SLF = CenterLf_y + r_e*sin(Theta_SLG) ;



%% %------------------------无障碍物RSR情况------------------------------------------
Centerfs1_dis = sqrt((CenterRs_x - CenterRf_x)^2 + (CenterRs_y - CenterRf_y)^2);
alphafs1_out = asin((r_e-r_s)/Centerfs1_dis);
beltfs1 = atan2((CenterRf_y - CenterRs_y),(CenterRf_x - CenterRs_x));
if isreal(alphafs1_out)

Thetafs_RSRG0 = beltfs1 + alphafs1_out + pi/2 ;
Thetafs_RSRF0 = beltfs1 + alphafs1_out + pi/2 ;
Thetafs_RSRG = mod(Thetafs_RSRG0 , 2*pi);
Thetafs_RSRF = mod(Thetafs_RSRF0 , 2*pi);

xfs_RSRG = CenterRs_x + r_s*cos(Thetafs_RSRG) ;
yfs_RSRG = CenterRs_y + r_s*sin(Thetafs_RSRG) ;%起点的切点
xfs_RSRF = CenterRf_x + r_e*cos(Thetafs_RSRF) ;
yfs_RSRF = CenterRf_y + r_e*sin(Thetafs_RSRF) ;

% plot(xfs_RSRG,yfs_RSRG,'s','MarkerSize',15,'MarkerFaceColor','b','MarkerEdgeColor','b');
% plot(xfs_RSRF,yfs_RSRF,'s','MarkerSize',15,'MarkerFaceColor','r','MarkerEdgeColor','r');

phyfs_RSRs = mod(Start_Theta - Thetafs_RSRG + pi/2 , 2*pi);
phyfs_RSRf  = mod(Thetafs_RSRF - End_Theta + 3*pi/2  , 2*pi);

Lengthfs_RSR = zeros(1,3);
Lengthfs_RSR(1) = phyfs_RSRs * r_s ;
Lengthfs_RSR(2) = sqrt((xfs_RSRF - xfs_RSRG)^2 + (yfs_RSRF - yfs_RSRG)^2);
Lengthfs_RSR(3) = phyfs_RSRf * r_e ;
LengthfsRSR = sum(Lengthfs_RSR) ;

Pointfs_RSR = [Start_X , xfs_RSRG , xfs_RSRF  , End_X;...
             Start_Y , yfs_RSRG , yfs_RSRF  , End_Y];

vthetafs_RSR=atan2((Pointfs_RSR(2,3)-Pointfs_RSR(2,2)),(Pointfs_RSR(1,3)-Pointfs_RSR(1,2)));

Phyfs_RSR = [phyfs_RSRs , phyfs_RSRf ];
Thetafs_RSR = [Start_Theta + pi/2 , Thetafs_RSRG ,Thetafs_RSRF];
Centerfs_RSR = [CenterRs_x,CenterRf_x;...
              CenterRs_y,CenterRf_y];

param1.point = Pointfs_RSR;
param1.length = Lengthfs_RSR;
param1.phy = Phyfs_RSR;
param1.theta = Thetafs_RSR;
param1.center = Centerfs_RSR;
param1.r = [r_s,r_e];
% param1.R = R;
param1.type = 1;
param1.vtheta = [Start_Theta,vthetafs_RSR,End_Theta];
param1.Length = sum(param1.length);
path1= DubinsPath(param1,Stepsize);
param1.path = path1;




end
%% ------------------------无障碍物RSL情况------------------------------------------
Centerfs2_dis = sqrt((CenterRs_x - CenterLf_x)^2 + (CenterRs_y - CenterLf_y)^2);
alphafs1_in = asin((r_e+r_s)/Centerfs2_dis);
beltfs2 = atan2((CenterLf_y - CenterRs_y),(CenterLf_x - CenterRs_x));
if isreal(alphafs1_in)

Thetafs_RSLG0 = beltfs2 - alphafs1_in + pi/2 ;
Thetafs_RSLF0 = beltfs2 - alphafs1_in + 3*pi/2 ;
Thetafs_RSLG = mod(Thetafs_RSLG0 , 2*pi);
Thetafs_RSLF = mod(Thetafs_RSLF0 , 2*pi);

xfs_RSLG = CenterRs_x + r_s*cos(Thetafs_RSLG) ;
yfs_RSLG = CenterRs_y + r_s*sin(Thetafs_RSLG) ;
xfs_RSLF = CenterLf_x + r_e*cos(Thetafs_RSLF) ;
yfs_RSLF = CenterLf_y + r_e*sin(Thetafs_RSLF) ;

phyfs_RSLs = mod(Start_Theta - Thetafs_RSLG + pi/2 , 2*pi);
phyfs_RSLf  = mod(End_Theta - Thetafs_RSLF + 3*pi/2  , 2*pi);

Lengthfs_RSL = zeros(1,3);
Lengthfs_RSL(1) = phyfs_RSLs * r_s ;
Lengthfs_RSL(2) = sqrt((xfs_RSLF - xfs_RSLG)^2 + (yfs_RSLF - yfs_RSLG)^2);
Lengthfs_RSL(3) = phyfs_RSLf * r_e ;
LengthfsRSL = sum(Lengthfs_RSL) ;

Pointfs_RSL = [Start_X , xfs_RSLG , xfs_RSLF  , End_X;...
             Start_Y , yfs_RSLG , yfs_RSLF  , End_Y];

vthetafs_RSL=atan2((Pointfs_RSL(2,3)-Pointfs_RSL(2,2)),(Pointfs_RSL(1,3)-Pointfs_RSL(1,2)));

Phyfs_RSL = [phyfs_RSLs , phyfs_RSLf ];
Thetafs_RSL = [Start_Theta + pi/2 , Thetafs_RSLG ,Thetafs_RSLF];
Centerfs_RSL = [CenterRs_x,CenterLf_x;...
              CenterRs_y,CenterLf_y];



param2.point = Pointfs_RSL;
param2.length = Lengthfs_RSL;
param2.phy = Phyfs_RSL;
param2.theta = Thetafs_RSL;
param2.center = Centerfs_RSL;
param2.r = [r_s,r_e];
% param2.R = R;
param2.type = 2;
param2.vtheta = [Start_Theta,vthetafs_RSL,End_Theta];
param2.Length = sum(param2.length);
path2 = DubinsPath(param2,Stepsize);
param2.path = path2;
end
 %% %------------------------无障碍物LSL情况------------------------------------------
Centerfs3_dis = sqrt((CenterLs_x - CenterLf_x)^2 + (CenterLs_y - CenterLf_y)^2);
alphafs2_out = asin((r_e-r_s)/Centerfs3_dis);
beltfs3 = atan2((CenterLf_y - CenterLs_y),(CenterLf_x - CenterLs_x));

if isreal(alphafs2_out)

Thetafs_LSLG0 = beltfs3 - alphafs2_out + 3*pi/2 ;
Thetafs_LSLF0 = beltfs3 - alphafs2_out + 3*pi/2 ;
Thetafs_LSLG = mod(Thetafs_LSLG0 , 2*pi);
Thetafs_LSLF = mod(Thetafs_LSLF0 , 2*pi);

xfs_LSLG = CenterLs_x + r_s*cos(Thetafs_LSLG) ;
yfs_LSLG = CenterLs_y + r_s*sin(Thetafs_LSLG) ;
xfs_LSLF = CenterLf_x + r_e*cos(Thetafs_LSLF) ;
yfs_LSLF = CenterLf_y + r_e*sin(Thetafs_LSLF) ;

phyfs_LSLs = mod(Thetafs_LSLG - Start_Theta + pi/2 , 2*pi);
phyfs_LSLf  = mod(End_Theta - Thetafs_LSLF + 3*pi/2  , 2*pi);

Lengthfs_LSL = zeros(1,3);
Lengthfs_LSL(1) = phyfs_LSLs * r_s ;
Lengthfs_LSL(2) = sqrt((xfs_LSLF - xfs_LSLG)^2 + (yfs_LSLF - yfs_LSLG)^2);
Lengthfs_LSL(3) = phyfs_LSLf * r_e ;
LengthfsLSL = sum(Lengthfs_LSL) ;

Pointfs_LSL = [Start_X , xfs_LSLG , xfs_LSLF  , End_X;...
             Start_Y , yfs_LSLG , yfs_LSLF  , End_Y];

vthetafs_LSL=atan2((Pointfs_LSL(2,3)-Pointfs_LSL(2,2)),(Pointfs_LSL(1,3)-Pointfs_LSL(1,2)));

Phyfs_LSL = [phyfs_LSLs , phyfs_LSLf ];
Thetafs_LSL = [Start_Theta - pi/2 , Thetafs_LSLG ,Thetafs_LSLF];
Centerfs_LSL = [CenterLs_x,CenterLf_x;...
              CenterLs_y,CenterLf_y]; 


param3.point = Pointfs_LSL;
param3.length = Lengthfs_LSL;
param3.theta = Thetafs_LSL;
param3.phy = Phyfs_LSL;
param3.center = Centerfs_LSL;
param3.r = [r_s,r_e];
% param3.R = R;
param3.type = 3;
param3.vtheta = [Start_Theta,vthetafs_LSL,End_Theta];
param3.Length = sum(param3.length);
path3= DubinsPath(param3,Stepsize);
param3.path = path3;
end
%% %------------------------无障碍物LSR情况------------------------------------------
Centerfs4_dis = sqrt((CenterLs_x - CenterRf_x)^2 + (CenterLs_y - CenterRf_y)^2);
alphafs2_in = asin((r_e+r_s)/Centerfs4_dis);
beltfs4= atan2((CenterRf_y - CenterLs_y),(CenterRf_x - CenterLs_x));

if isreal(alphafs2_in)

Thetafs_LSRG0 = beltfs4 + alphafs2_in + 3*pi/2 ;
Thetafs_LSRF0 = beltfs4 + alphafs2_in + pi/2 ;
Thetafs_LSRG = mod(Thetafs_LSRG0 , 2*pi);
Thetafs_LSRF = mod(Thetafs_LSRF0 , 2*pi);

xfs_LSRG = CenterLs_x + r_s*cos(Thetafs_LSRG) ;
yfs_LSRG = CenterLs_y + r_s*sin(Thetafs_LSRG) ;
xfs_LSRF = CenterRf_x + r_e*cos(Thetafs_LSRF) ;
yfs_LSRF = CenterRf_y + r_e*sin(Thetafs_LSRF) ;

phyfs_LSRs = mod(Thetafs_LSRG - Start_Theta+ pi/2 , 2*pi);
phyfs_LSRf  = mod(Thetafs_LSRF - End_Theta + 3*pi/2  , 2*pi);

Lengthfs_LSR = zeros(1,3);
Lengthfs_LSR(1) = phyfs_LSRs * r_s ;
Lengthfs_LSR(2) = sqrt((xfs_LSRF - xfs_LSRG)^2 + (yfs_LSRF - yfs_LSRG)^2);
Lengthfs_LSR(3) = phyfs_LSRf * r_e ;
LengthfsLSR = sum(Lengthfs_LSR) ;

Pointfs_LSR = [Start_X , xfs_LSRG , xfs_LSRF  , End_X;...
             Start_Y , yfs_LSRG , yfs_LSRF  , End_Y];

vthetafs_LSR=atan2((Pointfs_LSR(2,3)-Pointfs_LSR(2,2)),(Pointfs_LSR(1,3)-Pointfs_LSR(1,2)));


Phyfs_LSR = [phyfs_LSRs , phyfs_LSRf ];
Thetafs_LSR = [Start_Theta - pi/2 , Thetafs_LSRG ,Thetafs_LSRF];
Centerfs_LSR = [CenterLs_x,CenterRf_x;...
              CenterLs_y,CenterRf_y];

param4.point = Pointfs_LSR;
param4.length = Lengthfs_LSR;
param4.theta = Thetafs_LSR;
param4.phy = Phyfs_LSR;
param4.center = Centerfs_LSR;
param4.r = [r_s,r_e];
% param4.R = R;
param4.type = 4;
param4.vtheta = [Start_Theta,vthetafs_LSR,End_Theta];
param4.Length = sum(param4.length);
path4 = DubinsPath(param4,Stepsize);
param4.path = path4;

end
%% 挑选出最短路径











    
%     pathcell = {path1,path2,path3,path4};
    paramcell = {param1,param2,param3,param4};
    
%     Distance1 = zeros(length(path1(1,:)),1);
%     Distance2 = zeros(length(path2(1,:)),1);
%     Distance3 = zeros(length(path3(1,:)),1);
%     Distance4 = zeros(length(path4(1,:)),1);
    

    %% 找到最小值
    Dubins_Length1 = [LengthfsRSR , LengthfsRSL , LengthfsLSL , LengthfsLSR];

    index1 = find(Dubins_Length1 == min(Dubins_Length1),1); %最短索引



    param_best=paramcell{index1};

    if ~isempty(fieldnames(param1))
%         display(param1)
    param_all(1,1)=paramcell{1};
%     display(param_all)
    end
    if ~isempty(fieldnames(param2))
%         display(param2)
    param_all(1,2)=paramcell{2};
    end
    if ~isempty(fieldnames(param3))
%         display(param3)
    param_all(1,3)=paramcell{3};
    end
    if ~isempty(fieldnames(param4))
%         display(param4)        
    param_all(1,4)=paramcell{4};
    end


%     param_all(2,1)=paramcell{5};
%     param_all(2,2)=paramcell{6};
%     param_all(2,3)=paramcell{7};
%     param_all(2,4)=paramcell{8};

%     Dubins_Length1(index1) = max(Dubins_Length1)+1;%最短=最长+1
% 
%     index2 = find(Dubins_Length1 == min(Dubins_Length1),1); %第二短
%     Dubins_Length1(index2) = max(Dubins_Length1)+1;
% 
%     index3 = find(Dubins_Length1 == min(Dubins_Length1),1); %第三短
%     Dubins_Length1(index3) = max(Dubins_Length1)+1;
% 
%     index4 = find(Dubins_Length1 == min(Dubins_Length1),1); %第四短

%     Dubins_Length1 = [LengthfsRSR , LengthfsRSL , LengthfsLSL , LengthfsLSR];
    
    
%     error = 0.0001;

    %% 选择最优
%     if mDistance(index1) >= param.R - error%最短的是否不过障碍圆
%         param = paramcell{index1};
%     elseif mDistance(index2) >= param.R - error 
%         param = paramcell{index2}; 
%     elseif  mDistance(index3) >= param.R - error 
%         param = paramcell{index3}; 
%     elseif  mDistance(index4) >= param.R - error 
%         param = paramcell{index4}; 
%     end  
%     param.aim = Aim;

%     param.vtheta = [Start_Theta,End_Theta];
%     param.Length = sum(param.length);
end



