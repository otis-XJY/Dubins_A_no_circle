 function param = Dubins_obs(Start_Point,End_Point,Avoid_Center,r,R,Stepsize)
param.point = [];%切点集合
param.length = [];%每个路段长度
param.phy = [];%每个弧段的角度
param.center = [];%圆心
param.theta = [];
param.r = r;
param.R = R;
param.type = -1;%1-->RSR;2-->RSL;3-->LSL;4-->LSR 

Start_X = Start_Point(1);
Start_Y = Start_Point(2);
Start_Theta = Start_Point(3);

End_X = End_Point(1);
End_Y = End_Point(2);
End_Theta = End_Point(3);

Dx = Avoid_Center(1);
Dy = Avoid_Center(2);

%% 计算初始圆心
CenterRs_x = Start_X + r*cos(Start_Theta - pi/2);
CenterRs_y = Start_Y + r*sin(Start_Theta - pi/2);       %右转圆
CenterLs_x = Start_X + r*cos(Start_Theta + pi/2);
CenterLs_y = Start_Y + r*sin(Start_Theta + pi/2);       %左转圆

%% 计算末位置圆心
CenterRf_x = End_X + r*cos(End_Theta - pi/2);   
CenterRf_y = End_Y + r*sin(End_Theta - pi/2);           %右转圆
CenterLf_x = End_X + r*cos(End_Theta + pi/2);
CenterLf_y = End_Y + r*sin(End_Theta + pi/2);           %左转圆
%% 终点圆与目标区域的切点（右转圆） 
cR = sqrt((CenterRf_x - Dx)^2 + (CenterRf_y - Dy)^2);
alphaSR_out = asin((r-R)/cR);
beltSR = atan2((CenterRf_y - Dy ),(CenterRf_x - Dx  ));

Theta_SRG0 = beltSR + alphaSR_out + pi/2 ;
Theta_SRF0 = beltSR + alphaSR_out + pi/2 ;
Theta_SRG = mod(Theta_SRG0 , 2*pi);
Theta_SRF = mod(Theta_SRF0 , 2*pi);

x_SRG = Dx + R*cos(Theta_SRF) ;
y_SRG = Dy + R*sin(Theta_SRF) ;%障碍圆上的点
x_SRF = CenterRf_x + r*cos(Theta_SRG) ;
y_SRF = CenterRf_y + r*sin(Theta_SRG) ;%终点圆上的点

plot(x_SRG,y_SRG,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');
plot(x_SRF,y_SRF,'.','MarkerSize',25,'MarkerFaceColor','k','MarkerEdgeColor','k');

%% 终点圆与目标区域的切点（左转圆） 
cL = sqrt((CenterLf_x - Dx)^2 + (CenterLf_y - Dy)^2);
alphaSL_out = asin((r-R)/cL);
beltSL = atan2((CenterLf_y - Dy ),( CenterLf_x - Dx ));

Theta_SLG0 = beltSL - alphaSL_out + 3*pi/2 ;
Theta_SLF0 = beltSL - alphaSL_out + 3*pi/2 ;
Theta_SLG = mod(Theta_SLG0 , 2*pi);
Theta_SLF = mod(Theta_SLF0 , 2*pi);

x_SLG = Dx + R*cos(Theta_SLF) ;
y_SLG = Dy + R*sin(Theta_SLF) ;
x_SLF = CenterLf_x + r*cos(Theta_SLG) ;
y_SLF = CenterLf_y + r*sin(Theta_SLG) ;

%% 有障碍物情况：
%% 外公切 RSR_SR
c1 = sqrt((CenterRs_x - Dx)^2 + (CenterRs_y - Dy)^2);
alpha1_out = asin((R-r)/c1);
belt1 = atan2((Dy - CenterRs_y),(Dx - CenterRs_x));

Theta_RSRG0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRF0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRG = mod(Theta_RSRG0 , 2*pi);
Theta_RSRF = mod(Theta_RSRF0 , 2*pi);

x_RSRG = CenterRs_x + r*cos(Theta_RSRG) ;
y_RSRG = CenterRs_y + r*sin(Theta_RSRG) ;%起始点的右转圆切点
x_RSRF = Dx + R*cos(Theta_RSRF) ;
y_RSRF = Dy + R*sin(Theta_RSRF) ;%障碍圆的右

plot(x_RSRG,y_RSRG,'.','MarkerSize',25,'MarkerFaceColor','g','MarkerEdgeColor','g');
plot(x_RSRF,y_RSRF,'.','MarkerSize',25,'MarkerFaceColor','m','MarkerEdgeColor','m');

phy_RSRs = mod(Start_Theta - Theta_RSRG + pi/2 , 2*pi);
phy_RSRf = mod(Theta_RSRG - Theta_SRG , 2*pi);
phy_SRf  = mod(Theta_SRF - End_Theta + 3*pi/2  , 2*pi);

Length_RSR = zeros(1,5);
Length_RSR(1) = phy_RSRs * r ;
Length_RSR(2) = sqrt((x_RSRF - x_RSRG)^2 + (y_RSRF - y_RSRG)^2);
Length_RSR(3) = phy_RSRf * R ;
Length_RSR(4) = sqrt((x_SRF - x_SRG)^2 + (y_SRF - y_SRG)^2);
Length_RSR(5) = phy_SRf  * r ;
LengthRSR = sum(Length_RSR) ;

Point_RSR = [Start_X , x_RSRG , x_RSRF , x_SRG , x_SRF , End_X;...
             Start_Y , y_RSRG , y_RSRF , y_SRG , y_SRF , End_Y];
Phy_RSR = [phy_RSRs , phy_RSRf ,phy_SRf ];
Theta_RSR = [Start_Theta + pi/2 , Theta_RSRG ,Theta_RSRF , Theta_SRG , Theta_SRF ];
Center_RSR = [CenterRs_x,Dx,CenterRf_x;...
              CenterRs_y,Dy,CenterRf_y];

%% 内公切 RSL_SL

alpha1_in = asin((R+r)/c1);

Theta_RSLG0 = belt1 - alpha1_in + pi/2 ;
Theta_RSLF0 = belt1 - alpha1_in + 3*pi/2 ;
Theta_RSLG = mod(Theta_RSLG0 , 2*pi);
Theta_RSLF = mod(Theta_RSLF0 , 2*pi);

x_RSLG = CenterRs_x + r*cos(Theta_RSLG) ;
y_RSLG = CenterRs_y + r*sin(Theta_RSLG) ;
x_RSLF = Dx + R*cos(Theta_RSLF) ;
y_RSLF = Dy + R*sin(Theta_RSLF) ;

phy_RSLs = mod(Start_Theta - Theta_RSLG + pi/2 , 2*pi);
phy_RSLf = mod(Theta_SLG - Theta_RSLF , 2*pi);
phy_SLf  = mod(End_Theta - Theta_SLF   + 3*pi/2  , 2*pi);

Length_RSL = zeros(1,5);
Length_RSL(1) = phy_RSLs * r ;
Length_RSL(2) = sqrt((x_RSLF - x_RSLG)^2 + (y_RSLF - y_RSLG)^2);
Length_RSL(3) = phy_RSLf * R ;
Length_RSL(4) = sqrt((x_SLF - x_SLG)^2 + (y_SLF - y_SLG)^2);
Length_RSL(5) = phy_SLf  * r ;
LengthRSL = sum(Length_RSL) ;

Point_RSL = [Start_X , x_RSLG , x_RSLF , x_SLG , x_SLF , End_X;...
             Start_Y , y_RSLG , y_RSLF , y_SLG , y_SLF , End_Y];
         
Phy_RSL = [phy_RSLs , phy_RSLf ,phy_SLf];
Theta_RSL = [Start_Theta + pi/2 , Theta_RSLG ,Theta_RSLF , Theta_SLG , Theta_SLF ];
Center_RSL = [CenterRs_x,Dx,CenterLf_x;...
              CenterRs_y,Dy,CenterLf_y];      

%% 外公切 LSL_SL
c2 = sqrt((CenterLs_x - Dx)^2 + (CenterLs_y - Dy)^2);
alpha2_out = asin((R-r)/c2);
belt2 = atan2((Dy - CenterLs_y),(Dx - CenterLs_x));

Theta_LSLG0 = belt2 - alpha2_out + 3*pi/2 ;
Theta_LSLF0 = belt2 - alpha2_out + 3*pi/2 ;
Theta_LSLG = mod(Theta_LSLG0 , 2*pi);
Theta_LSLF = mod(Theta_LSLF0 , 2*pi);

x_LSLG = CenterLs_x + r*cos(Theta_LSLG) ;
y_LSLG = CenterLs_y + r*sin(Theta_LSLG) ;
x_LSLF = Dx + R*cos(Theta_LSLF) ;
y_LSLF = Dy + R*sin(Theta_LSLF) ;

phy_LSLs = mod(Theta_LSLG - Start_Theta + pi/2 , 2*pi);
phy_LSLf = mod(Theta_SLG - Theta_LSLF , 2*pi);
phy_SLf  = mod(End_Theta  - Theta_SLF + 3*pi/2  , 2*pi);

Length_LSL = zeros(1,5);
Length_LSL(1) = phy_LSLs * r ;
Length_LSL(2) = sqrt((x_LSLF - x_LSLG)^2 + (y_LSLF - y_LSLG)^2);
Length_LSL(3) = phy_LSLf * R ;
Length_LSL(4) = sqrt((x_SLF - x_SLG)^2 + (y_SLF - y_SLG)^2);
Length_LSL(5) = phy_SLf  * r ;
LengthLSL = sum(Length_LSL) ;

Point_LSL = [Start_X , x_LSLG , x_LSLF , x_SLG , x_SLF , End_X;...
             Start_Y , y_LSLG , y_LSLF , y_SLG , y_SLF , End_Y];
Phy_LSL = [phy_LSLs , phy_LSLf ,phy_SLf];
Theta_LSL = [Start_Theta - pi/2 , Theta_LSLG ,Theta_LSLF , Theta_SLG , Theta_SLF ];
Center_LSL = [CenterLs_x,Dx,CenterLf_x;...
              CenterLs_y,Dy,CenterLf_y];
%% 内公切 LSR_SR
alpha2_in = asin((R+r)/c2);

Theta_LSRG0 = belt2 + alpha2_in + 3*pi/2 ;
Theta_LSRF0 = belt2 + alpha2_in + pi/2 ;
Theta_LSRG = mod(Theta_LSRG0 , 2*pi);
Theta_LSRF = mod(Theta_LSRF0 , 2*pi);

x_LSRG = CenterLs_x + r*cos(Theta_LSRG) ;
y_LSRG = CenterLs_y + r*sin(Theta_LSRG) ;
x_LSRF = Dx + R*cos(Theta_LSRF) ;
y_LSRF = Dy + R*sin(Theta_LSRF) ;

phy_LSRs = mod(Theta_LSRG - Start_Theta + pi/2 , 2*pi);
phy_LSRf = mod(Theta_LSRF - Theta_SRG , 2*pi);
phy_SRf  = mod(Theta_SRF - End_Theta + 3*pi/2  , 2*pi);


Length_LSR = zeros(1,5);
Length_LSR(1) = phy_LSRs * r ;
Length_LSR(2) = sqrt((x_LSRF - x_LSRG)^2 + (y_LSRF - y_LSRG)^2);
Length_LSR(3) = phy_LSRf * R ;
Length_LSR(4) = sqrt((x_SRF - x_SRG)^2 + (y_SRF - y_SRG)^2);
Length_LSR(5) = phy_SRf  * r ;
LengthLSR = sum(Length_LSR) ;

Point_LSR = [Start_X , x_LSRG , x_LSRF , x_SRG , x_SRF , End_X;...
             Start_Y , y_LSRG , y_LSRF , y_SRG , y_SRF , End_Y];

Phy_LSR = [phy_LSRs , phy_LSRf ,phy_SRf];
Theta_LSR = [Start_Theta - pi/2 , Theta_LSRG ,Theta_LSRF , Theta_SRG , Theta_SRF ];
Center_LSR = [CenterLs_x,Dx,CenterRf_x;...
              CenterLs_y,Dy,CenterRf_y];

%% 

    param5.point = Point_RSR;
    param5.length = Length_RSR;
    param5.phy = Phy_RSR;
    param5.theta = Theta_RSR;
    param5.center = Center_RSR;
    param5.r = r;
    param5.R = R;
    param5.type = 5;

    param6.point = Point_RSL;
    param6.length = Length_RSL;
    param6.phy = Phy_RSL;
    param6.theta = Theta_RSL;
    param6.center = Center_RSL;
    param6.r = r;
    param6.R = R;
    param6.type = 6;

    param7.point = Point_LSL;
    param7.length = Length_LSL;
    param7.theta = Theta_LSL;
    param7.phy = Phy_LSL;
    param7.center = Center_LSL;
    param7.r = r;
    param7.R = R;
    param7.type = 7;
    
    param8.point = Point_LSR;
    param8.length = Length_LSR;
    param8.theta = Theta_LSR;
    param8.phy = Phy_LSR;
    param8.center = Center_LSR;
    param8.r = r;
    param8.R = R;
    param8.type = 8;
    
    path5 = DubinsPath_no_obs(param5,Stepsize);
    path6 = DubinsPath_no_obs(param6,Stepsize);
    path7 = DubinsPath_no_obs(param7,Stepsize);
    path8 = DubinsPath_no_obs(param8,Stepsize);
    

    param5.path = path5;
    param6.path = path6;
    param7.path = path7;
    param8.path = path8;
    
    pathcell = {path5,path6,path7,path8};
    paramcell = {param5,param6,param7,param8};

    Dubins_Length2 = [LengthRSR , LengthRSL , LengthLSL , LengthLSR];

    indexA = find(Dubins_Length2 == min(Dubins_Length2),1);
    
    error = 0.0001;

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

    param = paramcell{indexA};
    param.vtheta = [Start_Theta,End_Theta];
    param.Length = sum(param.length);
end

    

