function param=Dubins_Run(U,T,R_dubins,C_obs,step)
%% �������
Start_X = U(1);                                    %��ʼλ��x����
Start_Y = U(2);                                  %��ʼλ��y����
Start_Theta=transform_theta(U(3));                 %��ʼ�ٶ���x��ļн�
r = R_dubins;                                          %��С��ת�뾶
Dx = C_obs(1);                                         %Ŀ������Բ��x����
Dy = C_obs(2);                                         %Ŀ������Բ��y����
R = C_obs(3);                                          %Ŀ�������ϰ��뾶
Cx = T(1);                                         %Ŀ��λ��x����
Cy = T(2);                                        %Ŀ��λ��y����
Stepsize = step;                                 %����               
End_Theta = transform_theta(T(3));               %������ٶ���x��ļн�
End_Vx = cos(End_Theta);%����
End_Vy = sin(End_Theta);

End_X2 = [ (Cx*End_Vy^2 + Dx*End_Vx^2 + End_Vx*(- Cx^2*End_Vy^2 + 2*Cx*Cy*End_Vx*End_Vy + 2*Cx*Dx*End_Vy^2 - 2*Cx*Dy*End_Vx*End_Vy - Cy^2*End_Vx^2 - 2*Cy*Dx*End_Vx*End_Vy + 2*Cy*Dy*End_Vx^2 - Dx^2*End_Vy^2 + 2*Dx*Dy*End_Vx*End_Vy - Dy^2*End_Vx^2 + End_Vx^2*R^2 + End_Vy^2*R^2)^(1/2) - Cy*End_Vx*End_Vy + Dy*End_Vx*End_Vy)/(End_Vx^2 + End_Vy^2)
  (Cx*End_Vy^2 + Dx*End_Vx^2 - End_Vx*(- Cx^2*End_Vy^2 + 2*Cx*Cy*End_Vx*End_Vy + 2*Cx*Dx*End_Vy^2 - 2*Cx*Dy*End_Vx*End_Vy - Cy^2*End_Vx^2 - 2*Cy*Dx*End_Vx*End_Vy + 2*Cy*Dy*End_Vx^2 - Dx^2*End_Vy^2 + 2*Dx*Dy*End_Vx*End_Vy - Dy^2*End_Vx^2 + End_Vx^2*R^2 + End_Vy^2*R^2)^(1/2) - Cy*End_Vx*End_Vy + Dy*End_Vx*End_Vy)/(End_Vx^2 + End_Vy^2)
  ]; 
End_Y2 = [ (Cy*End_Vx^2 + Dy*End_Vy^2 + End_Vy*(- Cx^2*End_Vy^2 + 2*Cx*Cy*End_Vx*End_Vy + 2*Cx*Dx*End_Vy^2 - 2*Cx*Dy*End_Vx*End_Vy - Cy^2*End_Vx^2 - 2*Cy*Dx*End_Vx*End_Vy + 2*Cy*Dy*End_Vx^2 - Dx^2*End_Vy^2 + 2*Dx*Dy*End_Vx*End_Vy - Dy^2*End_Vx^2 + End_Vx^2*R^2 + End_Vy^2*R^2)^(1/2) - Cx*End_Vx*End_Vy + Dx*End_Vx*End_Vy)/(End_Vx^2 + End_Vy^2) 
 (Cy*End_Vx^2 + Dy*End_Vy^2 - End_Vy*(- Cx^2*End_Vy^2 + 2*Cx*Cy*End_Vx*End_Vy + 2*Cx*Dx*End_Vy^2 - 2*Cx*Dy*End_Vx*End_Vy - Cy^2*End_Vx^2 - 2*Cy*Dx*End_Vx*End_Vy + 2*Cy*Dy*End_Vx^2 - Dx^2*End_Vy^2 + 2*Dx*Dy*End_Vx*End_Vy - Dy^2*End_Vx^2 + End_Vx^2*R^2 + End_Vy^2*R^2)^(1/2) - Cx*End_Vx*End_Vy + Dx*End_Vx*End_Vy)/(End_Vx^2 + End_Vy^2)
 ];                                             %Ŀ�귽���ϵ��ӳ�����Բ�����������x����

plot(End_X2,End_Y2,'*','MarkerSize',30);


index1 = find(End_Vx*(End_X2 - Cx)<=0);
index2 = find(End_Vy*(End_Y2 - Cy)<=0) ;
End_X = End_X2(index1);
End_Y = End_Y2(index2);
%ѡ��ɽ���ķ���Ľ���=>Ŀ���


% syms x y;
% eq1 = (x-Dx)^2+(y-Dy)^2-R^2;
% eq2 = End_Vx*y-End_Vy*x+Cx*End_Vy-Cy*End_Vx;
% [End_X,End_Y]= solve(eq1,eq2,End_Vx*(x-Cx)<=0,End_Vy*(y-Cy)<=0); %������λ��
% End_X = double(End_X);
% End_Y = double(End_Y);
%% ���庯������
Start_Point = [Start_X,Start_Y,Start_Theta];
End_Point = [End_X,End_Y,End_Theta];

Avoid_Center = [Dx,Dy];
Aim = [Cx,Cy];

%% Ѱ��Dubins·��
param = DubinsAvoid(Start_Point, End_Point,Avoid_Center,Aim,r,R,Stepsize);
end