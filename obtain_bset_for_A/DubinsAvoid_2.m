% neg x c                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       function param = DubinsAvoid_2(Start_Point,End_Point,Avoid_Center,Aim,r,R,Stepsize)
syms x c
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
y_SRG = Dy + R*sin(Theta_SRF) ;
x_SRF = CenterRf_x + r*cos(Theta_SRG) ;
y_SRF = CenterRf_y + r*sin(Theta_SRG) ;

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

%------------------------无障碍物RSR情况------------------------------------------
Centerfs1_dis = sqrt((CenterRs_x - CenterRf_x)^2 + (CenterRs_y - CenterRf_y)^2);
alphafs1_out = asin((r-r)/Centerfs1_dis);
beltfs1 = atan2((CenterRf_y - CenterRs_y),(CenterRf_x - CenterRs_x));

Thetafs_RSRG0 = beltfs1 + alphafs1_out + pi/2 ;
Thetafs_RSRF0 = beltfs1 + alphafs1_out + pi/2 ;
Thetafs_RSRG = mod(Thetafs_RSRG0 , 2*pi);
Thetafs_RSRF = mod(Thetafs_RSRF0 , 2*pi);

xfs_RSRG = CenterRs_x + r*cos(Thetafs_RSRG) ;
yfs_RSRG = CenterRs_y + r*sin(Thetafs_RSRG) ;
xfs_RSRF = CenterRf_x + r*cos(Thetafs_RSRF) ;
yfs_RSRF = CenterRf_y + r*sin(Thetafs_RSRF) ;

phyfs_RSRs = mod(Start_Theta - Thetafs_RSRG + pi/2 , 2*pi);
phyfs_RSRf  = mod(Thetafs_RSRF - End_Theta + 3*pi/2  , 2*pi);

Lengthfs_RSR = zeros(1,3);
Lengthfs_RSR(1) = phyfs_RSRs * r ;
Lengthfs_RSR(2) = sqrt((xfs_RSRF - xfs_RSRG)^2 + (yfs_RSRF - yfs_RSRG)^2);
Lengthfs_RSR(3) = phyfs_RSRf * r ;
LengthfsRSR = sum(Lengthfs_RSR) ;

Pointfs_RSR = [Start_X , xfs_RSRG , xfs_RSRF  , End_X;...
             Start_Y , yfs_RSRG , yfs_RSRF  , End_Y];
Phyfs_RSR = [phyfs_RSRs , phyfs_RSRf ];
Thetafs_RSR = [Start_Theta + pi/2 , Thetafs_RSRG ,Thetafs_RSRF];
Centerfs_RSR = [CenterRs_x,Dx,CenterRf_x;...
              CenterRs_y,Dy,CenterRf_y];
%------------------------无障碍物RSL情况------------------------------------------
Centerfs2_dis = sqrt((CenterRs_x - CenterLf_x)^2 + (CenterRs_y - CenterLf_y)^2);
alphafs1_in = asin((r+r)/Centerfs2_dis);
beltfs2 = atan2((CenterLf_y - CenterRs_y),(CenterLf_x - CenterRs_x));

Thetafs_RSLG0 = beltfs2 - alphafs1_in + pi/2 ;
Thetafs_RSLF0 = beltfs2 - alphafs1_in + 3*pi/2 ;
Thetafs_RSLG = mod(Thetafs_RSLG0 , 2*pi);
Thetafs_RSLF = mod(Thetafs_RSLF0 , 2*pi);

xfs_RSLG = CenterRs_x + r*cos(Thetafs_RSLG) ;
yfs_RSLG = CenterRs_y + r*sin(Thetafs_RSLG) ;
xfs_RSLF = CenterLf_x + r*cos(Thetafs_RSLF) ;
yfs_RSLF = CenterLf_y + r*sin(Thetafs_RSLF) ;

phyfs_RSLs = mod(Start_Theta - Thetafs_RSLG + pi/2 , 2*pi);
phyfs_RSLf  = mod(End_Theta - Thetafs_RSLF + 3*pi/2  , 2*pi);

Lengthfs_RSL = zeros(1,3);
Lengthfs_RSL(1) = phyfs_RSLs * r ;
Lengthfs_RSL(2) = sqrt((xfs_RSLF - xfs_RSLG)^2 + (yfs_RSLF - yfs_RSLG)^2);
Lengthfs_RSL(3) = phyfs_RSLf * r ;
LengthfsRSL = sum(Lengthfs_RSL) ;

Pointfs_RSL = [Start_X , xfs_RSLG , xfs_RSLF  , End_X;...
             Start_Y , yfs_RSLG , yfs_RSLF  , End_Y];
Phyfs_RSL = [phyfs_RSLs , phyfs_RSLf ];
Thetafs_RSL = [Start_Theta + pi/2 , Thetafs_RSLG ,Thetafs_RSLF];
Centerfs_RSL = [CenterRs_x,Dx,CenterLf_x;...
              CenterRs_y,Dy,CenterLf_y];
%------------------------无障碍物LSL情况------------------------------------------
Centerfs3_dis = sqrt((CenterLs_x - CenterLf_x)^2 + (CenterLs_y - CenterLf_y)^2);
alphafs2_out = asin((r-r)/Centerfs3_dis);
beltfs3 = atan2((CenterLf_y - CenterLs_y),(CenterLf_x - CenterLs_x));

Thetafs_LSLG0 = beltfs3 - alphafs2_out + 3*pi/2 ;
Thetafs_LSLF0 = beltfs3 - alphafs2_out + 3*pi/2 ;
Thetafs_LSLG = mod(Thetafs_LSLG0 , 2*pi);
Thetafs_LSLF = mod(Thetafs_LSLF0 , 2*pi);

xfs_LSLG = CenterLs_x + r*cos(Thetafs_LSLG) ;
yfs_LSLG = CenterLs_y + r*sin(Thetafs_LSLG) ;
xfs_LSLF = CenterLf_x + r*cos(Thetafs_LSLF) ;
yfs_LSLF = CenterLf_y + r*sin(Thetafs_LSLF) ;

phyfs_LSLs = mod(Thetafs_LSLG - Start_Theta + pi/2 , 2*pi);
phyfs_LSLf  = mod(End_Theta - Thetafs_LSLF + 3*pi/2  , 2*pi);

Lengthfs_LSL = zeros(1,3);
Lengthfs_LSL(1) = phyfs_LSLs * r ;
Lengthfs_LSL(2) = sqrt((xfs_LSLF - xfs_LSLG)^2 + (yfs_LSLF - yfs_LSLG)^2);
Lengthfs_LSL(3) = phyfs_LSLf * r ;
LengthfsLSL = sum(Lengthfs_LSL) ;

Pointfs_LSL = [Start_X , xfs_LSLG , xfs_LSLF  , End_X;...
             Start_Y , yfs_LSLG , yfs_LSLF  , End_Y];
Phyfs_LSL = [phyfs_LSLs , phyfs_LSLf ];
Thetafs_LSL = [Start_Theta - pi/2 , Thetafs_LSLG ,Thetafs_LSLF];
Centerfs_LSL = [CenterLs_x,Dx,CenterLf_x;...
              CenterLs_y,Dy,CenterLf_y]; 
%------------------------无障碍物LSR情况------------------------------------------
Centerfs4_dis = sqrt((CenterLs_x - CenterRf_x)^2 + (CenterLs_y - CenterRf_y)^2);
alphafs2_in = asin((r+r)/Centerfs4_dis);
beltfs4= atan2((CenterRf_y - CenterLs_y),(CenterRf_x - CenterLs_x));

Thetafs_LSRG0 = beltfs4 + alphafs2_in + 3*pi/2 ;
Thetafs_LSRF0 = beltfs4 + alphafs2_in + pi/2 ;
Thetafs_LSRG = mod(Thetafs_LSRG0 , 2*pi);
Thetafs_LSRF = mod(Thetafs_LSRF0 , 2*pi);

xfs_LSRG = CenterLs_x + r*cos(Thetafs_LSRG) ;
yfs_LSRG = CenterLs_y + r*sin(Thetafs_LSRG) ;
xfs_LSRF = CenterRf_x + r*cos(Thetafs_LSRF) ;
yfs_LSRF = CenterRf_y + r*sin(Thetafs_LSRF) ;

phyfs_LSRs = mod(Thetafs_LSRG - Start_Theta+ pi/2 , 2*pi);
phyfs_LSRf  = mod(Thetafs_LSRF - End_Theta + 3*pi/2  , 2*pi);

Lengthfs_LSR = zeros(1,3);
Lengthfs_LSR(1) = phyfs_LSRs * r ;
Lengthfs_LSR(2) = sqrt((xfs_LSRF - xfs_LSRG)^2 + (yfs_LSRF - yfs_LSRG)^2);
Lengthfs_LSR(3) = phyfs_LSRf * r ;
LengthfsLSR = sum(Lengthfs_LSR) ;

Pointfs_LSR = [Start_X , xfs_LSRG , xfs_LSRF  , End_X;...
             Start_Y , yfs_LSRG , yfs_LSRF  , End_Y];
Phyfs_LSR = [phyfs_LSRs , phyfs_LSRf ];
Thetafs_LSR = [Start_Theta - pi/2 , Thetafs_LSRG ,Thetafs_LSRF];
Centerfs_LSR = [CenterLs_x,Dx,CenterRf_x;...
              CenterLs_y,Dy,CenterRf_y];
%% 外公切 RSR_SR
c1 = sqrt((CenterRs_x - Dx)^2 + (CenterRs_y - Dy)^2);
alpha1_out = asin((R-r)/c1);
belt1 = atan2((Dy - CenterRs_y),(Dx - CenterRs_x));

Theta_RSRG0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRF0 = belt1 + alpha1_out + pi/2 ;
Theta_RSRG = mod(Theta_RSRG0 , 2*pi);
Theta_RSRF = mod(Theta_RSRF0 , 2*pi);

x_RSRG = CenterRs_x + r*cos(Theta_RSRG) ;
y_RSRG = CenterRs_y + r*sin(Theta_RSRG) ;
x_RSRF = Dx + R*cos(Theta_RSRF) ;
y_RSRF = Dy + R*sin(Theta_RSRF) ;

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
%% 挑选出最短路径
    param1.point = Pointfs_RSR;
    param1.length = Lengthfs_RSR;
    param1.phy = Phyfs_RSR;
    param1.theta = Thetafs_RSR;
    param1.center = Centerfs_RSR;
    param1.r = r;
    param1.R = R;
    param1.type = 1;

    param2.point = Pointfs_RSL;
    param2.length = Lengthfs_RSL;
    param2.phy = Phyfs_RSL;
    param2.theta = Thetafs_RSL;
    param2.center = Centerfs_RSL;
    param2.r = r;
    param2.R = R;
    param2.type = 2;
    
    param3.point = Pointfs_LSL;
    param3.length = Lengthfs_LSL;
    param3.theta = Thetafs_LSL;
    param3.phy = Phyfs_LSL;
    param3.center = Centerfs_LSL;
    param3.r = r;
    param3.R = R;
    param3.type = 3;
    
    param4.point = Pointfs_LSR;
    param4.length = Lengthfs_LSR;
    param4.theta = Thetafs_LSR;
    param4.phy = Phyfs_LSR;
    param4.center = Centerfs_LSR;
    param4.r = r;
    param4.R = R;
    param4.type = 4;

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
    
    path1 = DubinsPath(param1,Stepsize);
    path2 = DubinsPath(param2,Stepsize);
    path3 = DubinsPath(param3,Stepsize);
    path4 = DubinsPath(param4,Stepsize);
    path5 = DubinsPath(param5,Stepsize);
    path6 = DubinsPath(param6,Stepsize);
    path7 = DubinsPath(param7,Stepsize);
    path8 = DubinsPath(param8,Stepsize);
    
    param1.path = path1;
    param2.path = path2;
    param3.path = path3;
    param4.path = path4;
    param5.path = path5;
    param6.path = path6;
    param7.path = path7;
    param8.path = path8;
    
    pathcell = {path1,path2,path3,path4,path5,path6,path7,path8};
    paramcell = {param1,param2,param3,param4,param5,param6,param7,param8};
    
    Distance1 = zeros(length(path1(1,:)),1);
    Distance2 = zeros(length(path2(1,:)),1);
    Distance3 = zeros(length(path3(1,:)),1);
    Distance4 = zeros(length(path4(1,:)),1);
    
    for i = 1:length(path1(1,:))
        Distance1(i) = sqrt((path1(1,i)-Dx)^2+(path1(2,i)-Dy)^2);
    end
    
    for i = 1:length(path2(1,:))
        Distance2(i) = sqrt((path2(1,i)-Dx)^2+(path2(2,i)-Dy)^2);
    end
    
    for i = 1:length(path3(1,:))
        Distance3(i) = sqrt((path3(1,i)-Dx)^2+(path3(2,i)-Dy)^2);
    end
    
    for i = 1:length(path4(1,:))
        Distance4(i) = sqrt((path4(1,i)-Dx)^2+(path4(2,i)-Dy)^2);
    end
    
    mDistance = [min(Distance1),min(Distance2),min(Distance3),min(Distance4)];
    
    Dubins_Length1 = [LengthfsRSR , LengthfsRSL , LengthfsLSL , LengthfsLSR];
    Dubins_Length2 = [LengthRSR , LengthRSL , LengthLSL , LengthLSR];
    index1 = find(Dubins_Length1 == min(Dubins_Length1),1); %最短索引
    Dubins_Length1(index1) = max(Dubins_Length1)+1;
    index2 = find(Dubins_Length1 == min(Dubins_Length1),1); %第二短
    Dubins_Length1(index2) = max(Dubins_Length1)+1;
    index3 = find(Dubins_Length1 == min(Dubins_Length1),1); %第三短
    Dubins_Length1(index3) = max(Dubins_Length1)+1;
    index4 = find(Dubins_Length1 == min(Dubins_Length1),1); %第四短
    Dubins_Length1 = [LengthfsRSR , LengthfsRSL , LengthfsLSL , LengthfsLSR];
    
    indexA = find(Dubins_Length2 == min(Dubins_Length2),1);
    
    error = 0.0001;
    if mDistance(index1) >= param.R - error
        param = paramcell{index1};
    elseif mDistance(index2) >= param.R - error && Dubins_Length1(index2)<= min(Dubins_Length2)
        param = paramcell{index2}; 
    elseif  Dubins_Length1(index2) > min(Dubins_Length2)
        param = paramcell{indexA+4};
    elseif  mDistance(index3) >= param.R - error && Dubins_Length1(index3)<= min(Dubins_Length2)
        param = paramcell{index3}; 
    elseif  Dubins_Length1(index3) > min(Dubins_Length2)
        param = paramcell{indexA+4};
    elseif  mDistance(index4) >= param.R - error && Dubins_Length1(index4)<= min(Dubins_Length2)
        param = paramcell{index4}; 
    elseif  mDistance(index4) < param.R || Dubins_Length1(index4) > min(Dubins_Length2)
        param = paramcell{indexA+4};
    end  
    param.aim = Aim;
    param.vtheta = [Start_Theta,End_Theta];
    param.Length = sum(param.length);
% end



