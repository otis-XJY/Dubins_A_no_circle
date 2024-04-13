function obs_know=obtain_obs_point(obs_param_known,num_obs)

obs_know=[];
for i=1:length(obs_param_known(:,1))
    [circle(:,1),circle(:,2)]=obtain_circle(obs_param_known(i,1),obs_param_known(i,2),obs_param_known(i,3),num_obs);
    obs_know=[obs_know;circle];
end


end