Uav = [2,2,45*pi/180];
    %130,101,23*pi/180;...
Target = [53,55,223*pi/180];
    %61,56,96*pi/180;...
R_dubins = 8;%��Сת��뾶
C_obs =[50,50,20];%�ϰ�Բ�Ĳ��� Բ��Xλ�ã�Yλ�ã��뾶
step = 0.1;%����
param_all = Dubins_Run(Uav,Target,R_dubins,C_obs,step);
DrawD(param_all,1, 1);