clear all;
clc;
close all;
%% ���ó�ʼ����
Uav = [2,2,45*pi/180;...
    130,101,23*pi/180;...
    100,3,130*pi/180;...
    %     4,105,200*pi/180;...
    112,121,324*pi/180;...
    102,106,316*pi/180];%UAV�ĳ�ʼ������Xλ�ã�Yλ�ã��ٶȷ���
Target = [50,45,45*pi/180;...
    61,56,96*pi/180;...
    45,60,196*pi/180;...
    55,68,24*pi/180];%TAR�ĳ�ʼ������Xλ�ã�Yλ�ã��ɹ�������
R_dubins = 5;%��Сת��뾶
R_max = 12;%ת��뾶����
R_step = 0.3;%�뾶��������
%��ȡĿ������λ�ã�X_obs,Y_obs����
[count_target, ~] = size(Target);
X_obs = Target(:, 1)' * ones(count_target,1) / count_target;
Y_obs = Target(:, 2)' * ones(count_target,1) / count_target;
C_obs =[X_obs,Y_obs,20];%�ϰ�Բ�Ĳ��� Xλ�ã�Yλ�ã��뾶
step = 0.1;%����
%% �������ֵ
[P,P1] = size(Uav);
[Q,Q1] = size(Target);
for i=1:P
    for j=1:Q
        param_all(i,j) = Dubins_Run(Uav(i,:),Target(j,:),R_dubins,C_obs,step);
        cost(i,j) = param_all(i,j).Length;
    end
end
[assignments, ~,~] = auction(cost, 0.1);

%% ��ͼ
for i=1:P
    if assignments(2,i)~=0
        DrawD(param_all(i,assignments(2,i)),i, assignments(2,i));
    end
end
