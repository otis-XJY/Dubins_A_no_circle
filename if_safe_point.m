function flag_safe=if_safe_point(obs_no_circle,center,R)

if R~=0
    th = 0:pi/4:2*pi;
    edge1x = center(1) + R*cos(th);
    edge1y = center(2)+  R*sin(th);
    edge1x=edge1x(1:8);
    edge1y=edge1y(1:8);
else
    edge1x=center(1);
    edge1y=center(2);
end


flag_safe=1;
for k=1:length(obs_no_circle)
   if ~isempty(obs_no_circle{k})
      x=obs_no_circle{k}(:,1);
      y=obs_no_circle{k}(:,2);
    
      [in,on] = inpolygon(edge1x,edge1y, x, y);
      if find(in&~on==1)
          flag_safe=0;
          break
      end
   end
end