function [angle_fov,angle_new,angle_CCW_CW]=constrain_angle(angle,angle_new,angle_CCW_CW,Stepsize_apf,r)
angle_ture1=angle+Stepsize_apf/r;
angle_ture1=transform_theta(mod(angle_ture1,2*pi));
angle_ture2=angle-Stepsize_apf/r;
angle_ture2=transform_theta(mod(angle_ture2,2*pi));
if abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle))))>Stepsize_apf/r/2
%                     text(200,200,[num2str(angle_new),'   ',num2str(angle)])
%                     pause(1)

%     angle_ture1=angle+Stepsize_apf/r;
%     angle_ture1=transform_theta(angle_ture1);
%     angle_ture2=angle-Stepsize_apf/r;
%     angle_ture2=transform_theta(angle_ture2);
    if length(angle_CCW_CW)>19 
        array=diff(angle_CCW_CW(end-19:end));
        if all(sign(array) == sign(array(1)))
            if all(array > 0)
                angle_new=angle_ture1;
                angle_fov=angle+Stepsize_apf/r/2;
                angle_fov=transform_theta(angle_fov);
            else
                angle_new=angle_ture2;
                angle_fov=angle-Stepsize_apf/r/2;
                angle_fov=transform_theta(angle_fov);
            end
        else
            if abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs( ...
                    transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture2))))
                angle_new=angle_ture1;
                angle_fov=angle+Stepsize_apf/r/2;
                angle_fov=transform_theta(angle_fov);
            else
                angle_new=angle_ture2;
                angle_fov=angle-Stepsize_apf/r/2;
                angle_fov=transform_theta(angle_fov);
            end
        end
    else
        if abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs( ...
                    transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture2))))
            angle_new=angle_ture1;
            angle_fov=angle+Stepsize_apf/r/2;
            angle_fov=transform_theta(angle_fov);
        else
            angle_new=angle_ture2;
            angle_fov=angle+Stepsize_apf/r/2;
            angle_fov=transform_theta(angle_fov);
        end
    end


    angle_CCW_CW=[angle_CCW_CW,angle_new];

%                     text(150,150, ['angle_new：',num2str(angle_new)]);
%                     text(150,130, ['angle：',num2str(angle)]);


%                     display([angle,angle_ture1,angle_ture2,angle_new])
%                     text(200,180,[num2str(angle_new),'  ',num2str(angle)])
%                     pause(1)
else
    angle_CCW_CW=[];

    cha=[abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle)))),
        abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture1)))),
        abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture2))))];
    [value,id_min]=min(cha);
    if id_min==1
        angle_new=angle;
        angle_fov=angle;
    elseif id_min==2
        angle_new=angle_ture1;
        angle_fov=angle+Stepsize_apf/r/2;
        angle_fov=transform_theta(angle_fov);      
    elseif id_min==3
        angle_new=angle_ture2;
        angle_fov=angle+Stepsize_apf/r/2;
        angle_fov=transform_theta(angle_fov);
    end

% % % %     if abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle))))<0.0001
% % % %             angle_new=angle;
% % % %             angle_fov=angle_fov;
% % % %     else
% % % %         if abs(transform_theta(abs(transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture1))))<abs(transform_theta(abs( ...
% % % %                         transform_theta_2pi(angle_new)-transform_theta_2pi(angle_ture2))))
% % % %                 angle_new=angle_ture1;
% % % %                 angle_fov=angle+Stepsize_apf/r/2;
% % % %                 angle_fov=transform_theta(angle_fov);
% % % %         else
% % % %                 angle_new=angle_ture2;
% % % %                 angle_fov=angle+Stepsize_apf/r/2;
% % % %                 angle_fov=transform_theta(angle_fov);
% % % %         end
% % % %     end
end

end