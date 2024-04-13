function Draw_obs_unknow_circle(obs_no_circle,obs_no_circle_in)

for i=1:length(obs_no_circle)
    if ~isempty(obs_no_circle{i})
      x=obs_no_circle{i}(:,1);
      y=obs_no_circle{i}(:,2);
      plot(x,y,'k-','LineWidth',2)
%       text(mean(x),mean(y),num2str(i))
    end
end

for i=1:length(obs_no_circle_in)
    if ~isempty(obs_no_circle_in{i})
      x=obs_no_circle_in{i}(:,1);
      y=obs_no_circle_in{i}(:,2);
      plot(x,y,'-','Color',[0.4660 0.6740 0.1880],'LineWidth',2)
      text(mean(x),mean(y),num2str(i))
    end
end



end