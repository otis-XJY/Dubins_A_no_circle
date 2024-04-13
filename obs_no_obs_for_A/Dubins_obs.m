  function [param_all,param_best] = Dubins_obs(Start_Point,End_Point,Avoid_Center,r_s,r_e,R,Stepsize)
param_best.point = [];%切点集合
param_best.length = [];%每个路段长度
param_best.phy = [];%每个弧段的角度
param_best.center = [];%圆心
param_best.theta = [];
param_best.r = [r_s,r_e];
param_best.R = R;
param_best.type = -1;%1-->RSR;2-->RSL;3-->LSL;4-->LSR 

param5=struct();
param6=struct();
param7=struct();
param8=struct();
LengthRSR=inf;
LengthRSL=inf;
LengthLSL=inf;
LengthLSR=inf;



Start_X = Start_Point(1);
Start_Y = Start_Point(2);
Start_Theta = Start_Point(3);

End_X = End_Point(1);
End_Y = End_Point(2);
End_Theta = End_Point(3);

Dx = Avoid_Center(1);
Dy = Avoid_Center(2);

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
%% 终点圆与目标区域的切点（右转圆） 
cR = sqrt((CenterRf_x - Dx)^2 + (CenterRf_y - Dy)^2);
alphaSR_out = asin((r_e-R)/cR);
beltSR = atan2((CenterRf_y - Dy ),(CenterRf_x - Dx  ));

Theta_SRG0 = beltSR + alphaSR_out + pi/2 ;
Theta_SRF0 = beltSR + alphaSR_out + pi/2 ;
Theta_SRG = mod(Theta_SRG0 , 2*pi);
Theta_SRF = mod(Theta_SRF0 , 2*pi);

x_SRG = Dx + R*cos(Theta_SRF) ;
y_SRG = Dy + R*sin(Theta_SRF) ;%障碍圆上的点
x_SRF = CenterRf_x + r_e*cos(Theta_SRG) ;
y_SRF = CenterRf_y + r_e*sin(Theta_SRG) ;%终点圆上的点

% plot(x_SRG,y_SRG,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');
% plot(x_SRF,y_SRF,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');

%% 终点圆与目标区域的切点（左转圆） 
cL = sqrt((CenterLf_x - Dx)^2 + (CenterLf_y - Dy)^2);
alphaSL_out = asin((r_e-R)/cL);
beltSL = atan2((CenterLf_y - Dy ),( CenterLf_x - Dx ));

Theta_SLG0 = beltSL - alphaSL_out + 3*pi/2 ;
Theta_SLF0 = beltSL - alphaSL_out + 3*pi/2 ;
Theta_SLG = mod(Theta_SLG0 , 2*pi);
Theta_SLF = mod(Theta_SLF0 , 2*pi);

x_SLG = Dx + R*cos(Theta_SLF) ;
y_SLG = Dy + R*sin(Theta_SLF) ;
x_SLF = CenterLf_x + r_e*cos(Theta_SLG) ;
y_SLF = CenterLf_y + r_e*sin(Theta_SLG) ;

%% 有障碍物情况：
%% 外公切 RSR_SR
c1 = sqrt((CenterRs_x - Dx)^2 + (CenterRs_y - Dy)^2);
alpha1_out = asin((R-r_s)/c1);
if isreal(alpha1_out)
belt1 = atan2((Dy - CenterRs_y),(Dx - CenterRs_x));

Theta_RSRG0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRF0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRG = mod(Theta_RSRG0 , 2*pi);
Theta_RSRF = mod(Theta_RSRF0 , 2*pi);

x_RSRG = CenterRs_x + r_s*cos(Theta_RSRG) ;
y_RSRG = CenterRs_y + r_s*sin(Theta_RSRG) ;%起始点的右转圆切点
x_RSRF = Dx + R*cos(Theta_RSRF) ;
y_RSRF = Dy + R*sin(Theta_RSRF) ;%障碍圆的右

% plot(x_RSRG,y_RSRG,'.','MarkerSize',25,'MarkerFaceColor','g','MarkerEdgeColor','g');
% plot(x_RSRF,y_RSRF,'.','MarkerSize',25,'MarkerFaceColor','m','MarkerEdgeColor','m');

phy_RSRs = mod(Start_Theta - Theta_RSRG + pi/2 , 2*pi);
phy_RSRf = mod(Theta_RSRG - Theta_SRG , 2*pi);
phy_SRf  = mod(Theta_SRF - End_Theta + 3*pi/2  , 2*pi);

Length_RSR = zeros(1,5);
Length_RSR(1) = phy_RSRs * r_s ;
Length_RSR(2) = sqrt((x_RSRF - x_RSRG)^2 + (y_RSRF - y_RSRG)^2);
Length_RSR(3) = phy_RSRf * R ;
Length_RSR(4) = sqrt((x_SRF - x_SRG)^2 + (y_SRF - y_SRG)^2);
Length_RSR(5) = phy_SRf  * r_e ;
LengthRSR = sum(Length_RSR) ;

Point_RSR = [Start_X , x_RSRG , x_RSRF , x_SRG , x_SRF , End_X;...
             Start_Y , y_RSRG , y_RSRF , y_SRG , y_SRF , End_Y];

vtheta_RSR=zeros(1,2);
vtheta_RSR(1)=atan2((Point_RSR(2,3)-Point_RSR(2,2)),(Point_RSR(1,3)-Point_RSR(1,2)));
vtheta_RSR(2)=atan2((Point_RSR(2,5)-Point_RSR(2,4)),(Point_RSR(1,5)-Point_RSR(1,4)));

% quiver(Point_RSR(1,2),Point_RSR(2,2),15*cos(vtheta_RSR),15*sin(vtheta_RSR),'Color','r','LineWidth',1);
% hold on
Phy_RSR = [phy_RSRs , phy_RSRf ,phy_SRf ];
Theta_RSR = [Start_Theta + pi/2 , Theta_RSRG ,Theta_RSRF , Theta_SRG , Theta_SRF ];
Center_RSR = [CenterRs_x,Dx,CenterRf_x;...
              CenterRs_y,Dy,CenterRf_y];

param5.point = Point_RSR;
param5.length = Length_RSR;
param5.phy = Phy_RSR;
param5.theta = Theta_RSR;
param5.center = Center_RSR;
param5.r = [r_s,r_e];
param5.R = R;
param5.type = 5;
param5.vtheta = [Start_Theta,vtheta_RSR,End_Theta];
param5.Length = sum(param5.length);
path5 = DubinsPath(param5,Stepsize);
param5.path = path5;

end

%% 内公切 RSL_SL

alpha1_in = asin((R+r_s)/c1);
if isreal(alpha1_in)
Theta_RSLG0 = belt1 - alpha1_in + pi/2 ;
Theta_RSLF0 = belt1 - alpha1_in + 3*pi/2 ;
Theta_RSLG = mod(Theta_RSLG0 , 2*pi);
Theta_RSLF = mod(Theta_RSLF0 , 2*pi);

x_RSLG = CenterRs_x + r_s*cos(Theta_RSLG) ;
y_RSLG = CenterRs_y + r_s*sin(Theta_RSLG) ;
x_RSLF = Dx + R*cos(Theta_RSLF) ;
y_RSLF = Dy + R*sin(Theta_RSLF) ;

phy_RSLs = mod(Start_Theta - Theta_RSLG + pi/2 , 2*pi);
phy_RSLf = mod(Theta_SLG - Theta_RSLF , 2*pi);
phy_SLf  = mod(End_Theta - Theta_SLF   + 3*pi/2  , 2*pi);

Length_RSL = zeros(1,5);
Length_RSL(1) = phy_RSLs * r_s ;
Length_RSL(2) = sqrt((x_RSLF - x_RSLG)^2 + (y_RSLF - y_RSLG)^2);
Length_RSL(3) = phy_RSLf * R ;
Length_RSL(4) = sqrt((x_SLF - x_SLG)^2 + (y_SLF - y_SLG)^2);
Length_RSL(5) = phy_SLf  * r_e ;
LengthRSL = sum(Length_RSL) ;

Point_RSL = [Start_X , x_RSLG , x_RSLF , x_SLG , x_SLF , End_X;...
             Start_Y , y_RSLG , y_RSLF , y_SLG , y_SLF , End_Y];

vtheta_RSL=zeros(1,2);
vtheta_RSL(1)=atan2((Point_RSL(2,3)-Point_RSL(2,2)),(Point_RSL(1,3)-Point_RSL(1,2)));
vtheta_RSL(2)=atan2((Point_RSL(2,5)-Point_RSL(2,4)),(Point_RSL(1,5)-Point_RSL(1,4)));

Phy_RSL = [phy_RSLs , phy_RSLf ,phy_SLf];
Theta_RSL = [Start_Theta + pi/2 , Theta_RSLG ,Theta_RSLF , Theta_SLG , Theta_SLF ];
Center_RSL = [CenterRs_x,Dx,CenterLf_x;...
              CenterRs_y,Dy,CenterLf_y];    



param6.point = Point_RSL;
param6.length = Length_RSL;
param6.phy = Phy_RSL;
param6.theta = Theta_RSL;
param6.center = Center_RSL;
param6.r = [r_s,r_e];
param6.R = R;
param6.type = 6;
param6.vtheta = [Start_Theta,vtheta_RSL,End_Theta];
param6.Length = sum(param6.length);
    
path6 = DubinsPath(param6,Stepsize);
param6.path = path6;
end

%% 外公切 LSL_SL
c2 = sqrt((CenterLs_x - Dx)^2 + (CenterLs_y - Dy)^2);
alpha2_out = asin((R-r_s)/c2);
if isreal(alpha2_out)

belt2 = atan2((Dy - CenterLs_y),(Dx - CenterLs_x));

Theta_LSLG0 = belt2 - alpha2_out + 3*pi/2 ;
Theta_LSLF0 = belt2 - alpha2_out + 3*pi/2 ;
Theta_LSLG = mod(Theta_LSLG0 , 2*pi);
Theta_LSLF = mod(Theta_LSLF0 , 2*pi);

x_LSLG = CenterLs_x + r_s*cos(Theta_LSLG) ;
y_LSLG = CenterLs_y + r_s*sin(Theta_LSLG) ;
x_LSLF = Dx + R*cos(Theta_LSLF) ;
y_LSLF = Dy + R*sin(Theta_LSLF) ;

phy_LSLs = mod(Theta_LSLG - Start_Theta + pi/2 , 2*pi);
phy_LSLf = mod(Theta_SLG - Theta_LSLF , 2*pi);
phy_SLf  = mod(End_Theta  - Theta_SLF + 3*pi/2  , 2*pi);

Length_LSL = zeros(1,5);
Length_LSL(1) = phy_LSLs * r_s ;
Length_LSL(2) = sqrt((x_LSLF - x_LSLG)^2 + (y_LSLF - y_LSLG)^2);
Length_LSL(3) = phy_LSLf * R ;
Length_LSL(4) = sqrt((x_SLF - x_SLG)^2 + (y_SLF - y_SLG)^2);
Length_LSL(5) = phy_SLf  * r_e ;
LengthLSL = sum(Length_LSL) ;

Point_LSL = [Start_X , x_LSLG , x_LSLF , x_SLG , x_SLF , End_X;...
             Start_Y , y_LSLG , y_LSLF , y_SLG , y_SLF , End_Y];

vtheta_LSL=zeros(1,2);
vtheta_LSL(1)=atan2((Point_LSL(2,3)-Point_LSL(2,2)),(Point_LSL(1,3)-Point_LSL(1,2)));
vtheta_LSL(2)=atan2((Point_LSL(2,5)-Point_LSL(2,4)),(Point_LSL(1,5)-Point_LSL(1,4)));

Phy_LSL = [phy_LSLs , phy_LSLf ,phy_SLf];
Theta_LSL = [Start_Theta - pi/2 , Theta_LSLG ,Theta_LSLF , Theta_SLG , Theta_SLF ];
Center_LSL = [CenterLs_x,Dx,CenterLf_x;...
              CenterLs_y,Dy,CenterLf_y];

param7.point = Point_LSL;
param7.length = Length_LSL;
param7.theta = Theta_LSL;
param7.phy = Phy_LSL;
param7.center = Center_LSL;
param7.r = [r_s,r_e];
param7.R = R;
param7.type = 7;
param7.vtheta = [Start_Theta,vtheta_LSL,End_Theta];
param7.Length = sum(param7.length);
path7 = DubinsPath(param7,Stepsize);
param7.path = path7;

end

%% 内公切 LSR_SR
alpha2_in = asin((R+r_s)/c2);
if isreal(alpha2_in)

Theta_LSRG0 = belt2 + alpha2_in + 3*pi/2 ;
Theta_LSRF0 = belt2 + alpha2_in + pi/2 ;
Theta_LSRG = mod(Theta_LSRG0 , 2*pi);
Theta_LSRF = mod(Theta_LSRF0 , 2*pi);

x_LSRG = CenterLs_x + r_s*cos(Theta_LSRG) ;
y_LSRG = CenterLs_y + r_s*sin(Theta_LSRG) ;
x_LSRF = Dx + R*cos(Theta_LSRF) ;
y_LSRF = Dy + R*sin(Theta_LSRF) ;

phy_LSRs = mod(Theta_LSRG - Start_Theta + pi/2 , 2*pi);
phy_LSRf = mod(Theta_LSRF - Theta_SRG , 2*pi);
phy_SRf  = mod(Theta_SRF - End_Theta + 3*pi/2  , 2*pi);


Length_LSR = zeros(1,5);
Length_LSR(1) = phy_LSRs * r_s ;
Length_LSR(2) = sqrt((x_LSRF - x_LSRG)^2 + (y_LSRF - y_LSRG)^2);
Length_LSR(3) = phy_LSRf * R ;
Length_LSR(4) = sqrt((x_SRF - x_SRG)^2 + (y_SRF - y_SRG)^2);
Length_LSR(5) = phy_SRf  * r_e ;
LengthLSR = sum(Length_LSR) ;

Point_LSR = [Start_X , x_LSRG , x_LSRF , x_SRG , x_SRF , End_X;...
             Start_Y , y_LSRG , y_LSRF , y_SRG , y_SRF , End_Y];

vtheta_LSR=zeros(1,2);
vtheta_LSR(1)=atan2((Point_LSR(2,3)-Point_LSR(2,2)),(Point_LSR(1,3)-Point_LSR(1,2)));
vtheta_LSR(2)=atan2((Point_LSR(2,5)-Point_LSR(2,4)),(Point_LSR(1,5)-Point_LSR(1,4)));

Phy_LSR = [phy_LSRs , phy_LSRf ,phy_SRf];
Theta_LSR = [Start_Theta - pi/2 , Theta_LSRG ,Theta_LSRF , Theta_SRG , Theta_SRF ];
Center_LSR = [CenterLs_x,Dx,CenterRf_x;...
              CenterLs_y,Dy,CenterRf_y];

param8.point = Point_LSR;
param8.length = Length_LSR;
param8.theta = Theta_LSR;
param8.phy = Phy_LSR;
param8.center = Center_LSR;
param8.r = [r_s,r_e];
param8.R = R;
param8.type = 8;
param8.vtheta = [Start_Theta,vtheta_LSR,End_Theta];
param8.Length = sum(param8.length);
path8 = DubinsPath(param8,Stepsize);
param8.path = path8;
end

%% 

    
%     pathcell = {path5,path6,path7,path8};
    paramcell = {param5,param6,param7,param8};

    Dubins_Length2 = [LengthRSR , LengthRSL , LengthLSL , LengthLSR];

    indexA = find(Dubins_Length2 == min(Dubins_Length2),1);

    param_best=paramcell{indexA};

    
    if ~isempty(fieldnames(param5))
%         display(param6)
    param_all(1,1)=paramcell{1};
    end
    if ~isempty(fieldnames(param6))
%         display(param6)
    param_all(1,2)=paramcell{2};
    end
    if ~isempty(fieldnames(param7))
%         display(param6)
    param_all(1,3)=paramcell{3};
    end
    if ~isempty(fieldnames(param8))
%         display(param6)        
    param_all(1,4)=paramcell{4};
    end
%     param_all(2,1)=paramcell{5};
%     param_all(2,2)=paramcell{6};
%     param_all(2,3)=paramcell{7};
%     param_all(2,4)=paramcell{8};
    
%     error = 0.0001;

    %% 选择最优
%     if mDistance(index1) >= param.R - error%最短的是否不过障碍圆
%         param = paramcell{index1};
%     elseif mDistance(index2) >= param.R - error && Dubins_Length1(index2)<= min(Dubins_Length2)
%         param = paramcell{index2}; 
%     elseif  Dubins_Length1(index2) > min(Dubins_Length2)
%         param = paramcell{indexA+4};
%     elseif  mDistance(index3) >= param.R - error && Dubins_Length1(index3)<= min(Dubins_Length2)
%         param = paramcell{index3}; 
%     elseif  Dubins_Length1(index3) > min(Dubins_Length2)
%         param = paramcell{indexA+4};
%     elseif  mDistance(index4) >= param.R - error && Dubins_Length1(index4)<= min(Dubins_Length2)
%         param = paramcell{index4}; 
%     elseif  mDistance(index4) < param.R || Dubins_Length1(index4) > min(Dubins_Length2)
%         param = paramcell{indexA+4};
%     end  


end

    

