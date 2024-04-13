function param=obtain_vtheta_path(param)

vtheta_all=[];
for i=1:length(param)
    if isfield(param(i),'path')
        for j=1:length(param(i).path)
            if j==1
                vtheta_all=[param(i).vtheta(1)];
                for k=1:length(param(i).path{j}(1,:))
%                     param(i).center(:,1)
%                     param(i).path{j}(:,k)
%                     param(i).vtheta(1)
                    vtheta=obtain_vtheta(param(i).center(:,1),param(i).path{j}(:,k),vtheta_all(end));
                    vtheta_all=[vtheta_all,vtheta];                    
                end
                vtheta_all=vtheta_all(2:end);
                param(i).vtheta_all{j}=vtheta_all;
            elseif j==2
                param(i).vtheta_all{j}=param(i).vtheta(2)*ones(1,length(param(i).path{j}));
            elseif j==3
                 vtheta_all=[param(i).vtheta(2)];
                for k=1:length(param(i).path{j}(1,:))
                    vtheta=obtain_vtheta(param(i).center(:,2),param(i).path{j}(:,k),vtheta_all(end));
                    vtheta_all=[vtheta_all,vtheta];  
                end
                vtheta_all=vtheta_all(2:end);
                param(i).vtheta_all{j}=vtheta_all;
            elseif j==4
                param(i).vtheta_all{j}=param(i).vtheta(3)*ones(1,length(param(i).path{j}));

          elseif j==5
              vtheta_all=[param(i).vtheta(3)];
                for k=1:length(param(i).path{j}(1,:))
                    vtheta=obtain_vtheta(param(i).center(:,3),param(i).path{j}(:,k),vtheta_all(end));
                    vtheta_all=[vtheta_all,vtheta]; 
                    
                end
                vtheta_all=vtheta_all(2:end);
                    param(i).vtheta_all{j}=vtheta_all;
            end

        end
    end
end
end