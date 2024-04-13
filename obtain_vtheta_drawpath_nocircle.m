function [draw_path,vtheta_all]=obtain_vtheta_drawpath_nocircle(final_path)

draw_path=[];
vtheta_all=[0];
for i=1:length(final_path)
    if isfield(final_path{i},'path')
        for j=1:length(final_path{i}.path)
            draw_path=[draw_path,final_path{i}.path{j}];
       end
    end

    if isfield(final_path{i},'vtheta_all')
        for j=1:length(final_path{i}.vtheta_all)
            vtheta_all=[vtheta_all,final_path{i}.vtheta_all{j}];
       end
    end




end
end