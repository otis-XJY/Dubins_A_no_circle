Uav = [2,2,45*pi/180];
    %130,101,23*pi/180;...
Target = [53,55,223*pi/180];
    %61,56,96*pi/180;...
R_dubins = 8;%最小转弯半径
C_obs =[50,50,20];%障碍圆的参数 圆心X位置，Y位置，半径
step = 0.1;%步长
param_all = Dubins_Run(Uav,Target,R_dubins,C_obs,step);
DrawD(param_all,1, 1);