function h=obtain_h(start,goal)
% h =10* abs(  start(1)-goal(1)  )+10*abs(  start(2)-goal(2)  );
h=1.414*sqrt((  start(1)-goal(1)  )^2+(start(2)-goal(2))^2);
end
