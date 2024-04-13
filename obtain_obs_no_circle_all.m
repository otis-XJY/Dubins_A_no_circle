function [outline_all,obs_no_circle,obs_no_circle_in]=obtain_obs_no_circle_all(num_obs_nocircle,mapsize,R,num_steps,Start_Point,End_Point,sure)
outline_all=[];
obs_no_circle=[];
obs_no_circle_in=[];
for i=1:num_obs_nocircle
    [outline,vertex,expandedPolygon]=obtain_obs_nocircle(mapsize,R,num_steps,Start_Point,End_Point,sure);
    if ~isempty(outline)
        outline=[outline,ones(length(outline(:,1)),1)*i];
        outline_all=[outline_all;outline];
        obs_no_circle{i}=expandedPolygon;
        obs_no_circle_in{i}=vertex;
    end
end
