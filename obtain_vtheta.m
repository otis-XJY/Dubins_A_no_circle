function vtheta=obtain_vtheta(center,pos_now,vtheta_old)


x=pos_now(1)-center(1);
y=pos_now(2)-center(2);

% v=(-y,x);
% v=(y,-x);
% vtheta1=vpa(atan2(x,-y),3);
% vtheta2=vpa(atan2(-x,y),3);
% vtheta_old=vpa(vtheta_old,3);


vtheta1=atan2(x,-y);
vtheta2=atan2(-x,y);
% vtheta_old=vtheta_old;
if vtheta1<0
    vtheta1_=vtheta1+2*pi;
else
    vtheta1_=vtheta1;   
end

if vtheta2<0
    vtheta2_=vtheta2+2*pi;
else
    vtheta2_=vtheta2;
end

if vtheta_old<0
    vtheta_old_=vtheta_old+2*pi;
else
    vtheta_old_=vtheta_old;
end
% [vtheta1 vtheta2 vtheta_old]
% fileID = fopen('exp.txt','a');
% % fprintf(fileID,'%6s %12s\n','x','exp(x)');
% fprintf(fileID,'%6.6f %12.6f %12.6f\n',A);
% fclose(fileID);



cha1=abs(vtheta1_-vtheta_old_);
cha2=abs(vtheta2_-vtheta_old_);
% if cha1>2*3.14
%     cha1=cha1-2*3.14;
% end
% 
% if cha2>2*3.14
%     cha2=cha2-2*3.14;
% end
% 
% 
cha=[cha1,cha2];
if vtheta_old_<0.01 && 2*pi-max([vtheta1_,vtheta2_])<0.01
id=find(cha==max(cha));
% vtheta_old_
% [vtheta_old_,vtheta1_,vtheta2_]
% cha
else
id=find(cha==min(cha));
end


if 2*pi-vtheta_old_<0.01 && min([vtheta1_,vtheta2_])<0.01
id=find(cha==max(cha));
% cha
else
id=find(cha==min(cha));
end


all_theta=[vtheta1,vtheta2];


vtheta=all_theta(id);

% if cha1<3.14/2
%     vtheta=vtheta1;
% elseif cha2<3.14/2
%     vtheta=vtheta2;
% end

% vtheta_old vtheta];






% display(A)

end

