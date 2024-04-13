function [open_f,open]=check_in_open_new(param_safe,End_Point,open_f,open,close,obs_num_)
pos_id_all=[];
for i=1:length(open)
    pos_id_all=[pos_id_all,open{i}.pos_id];
end




if obs_num_<=length(param_safe.R)
    if ~isempty(param_safe.R{obs_num_})
        for i=1:length(param_safe.R{obs_num_})
            
            param=param_safe.R{obs_num_}(i);
            if ~isempty(param.pos_id)
    
                pos_in_open=find(pos_id_all==param.pos_id);
                num=length(param.path);
                if ~isempty(pos_in_open)
                    for j=1:length(pos_in_open)
                        


                        
                        Length=sum(param.length(1:num-3));
                        new_g=close{end}.g+Length;
                        new_f=new_g+obtain_h(param.point(:,num+1-3)',End_Point);
                
                        
                        if new_f<open{pos_in_open(j)}.f
                            display([new_f,open{pos_in_open(j)}.f,open{pos_in_open(j)}.pos_id,open{pos_in_open(j)}.parent_id])
                            display('change');
%                             param
%                             num
%                             pos_in_open(j)

                            open{pos_in_open(j)}.point=param.point(:,1:num-2);
%                             open{pos_in_open(j)}.point
                            open{pos_in_open(j)}.vtheta=param.vtheta(2:num-2-1);
                            open{pos_in_open(j)}.vtheta_all=param.vtheta_all(1:num-3);
                        
                        %     node.Length=sum(param.length(1:2));
                        
                            open{pos_in_open(j)}.g=new_g;
                        %     node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                            open{pos_in_open(j)}.f=open{pos_in_open(j)}.g+obtain_h(open{pos_in_open(j)}.point(:,end)',End_Point);
                        %         node.f=close{end}.g+node.g+sum(param.length(3:5));
                            open{pos_in_open(j)}.path=param.path(1:num-3);
%                             open{pos_in_open(j)}.path{2}=param.path{2};
                            open{pos_in_open(j)}.R=param.R;
                            open{pos_in_open(j)}.r=param.r;%%[r_s,r_e]
                            open{pos_in_open(j)}.center=param.center(1:num-3-2);
                            open{pos_in_open(j)}.pos_id=param.pos_id;
                            open{pos_in_open(j)}.parent_id=param.parent_id;
                            open_f(pos_in_open(j))=open{pos_in_open(j)}.f;
                        else
                            display('ignore')
                        end
                    end
                %     display('修改');
                %     display(pos_in_open);
                else
                    node.point=param.point(:,1:num-2);
                    node.vtheta=param.vtheta(2:num-2-1);
                    node.vtheta_all=param.vtheta_all(1:num-3);
                
                %     node.Length=sum(param.length(1:2));
                
                    node.g=close{end}.g+sum(param.length(1:num-3));
                %     node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                    node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                %         node.f=close{end}.g+node.g+sum(param.length(3:5));
                    node.path=param.path(1:num-3);
%                     node.path{2}=param.path{2};
                    node.R=param.R;
                    node.r=param.r;%%[r_s,r_e]
                    node.center=param.center(1:num-3-2);
                    node.pos_id=param.pos_id;
                    node.parent_id=param.parent_id;
                    open_f=[open_f,node.f];
                    open{end+1}=node;
                end
            end
        end
    end
end









if obs_num_<=length(param_safe.r)
    if ~isempty(param_safe.r{obs_num_})
        for i=1:length(param_safe.r{obs_num_})
            param=param_safe.r{obs_num_}(i);
            if ~isempty(param.pos_id)
                pos_in_open=find(pos_id_all==param.pos_id);
                num=length(param.path);
                if ~isempty(pos_in_open)
                    for j=1:length(pos_in_open)
                        Length=sum(param.length(1:2));
                        new_g=close{end}.g+Length;
                        new_f=new_g+obtain_h(param.point(:,3)',End_Point);
                
%                         display([new_f,open{pos_in_open(j)}.f,open{pos_in_open(j)}.pos_id,open{pos_in_open(j)}.parent_id])
                        if new_f<open{pos_in_open(j)}.f


                            display([new_f,open{pos_in_open(j)}.f,open{pos_in_open(j)}.pos_id,open{pos_in_open(j)}.parent_id])
                            display('change');






                            open{pos_in_open(j)}.point=param.point(:,1:num-2);
%                             open{pos_in_open(j)}.point
                            open{pos_in_open(j)}.vtheta=param.vtheta(2:num-2-1);
                            open{pos_in_open(j)}.vtheta_all=param.vtheta_all(1:num-3);
                        
                        %     node.Length=sum(param.length(1:2));
                        
                            open{pos_in_open(j)}.g=new_g;
                        %     node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                            open{pos_in_open(j)}.f=open{pos_in_open(j)}.g+obtain_h(open{pos_in_open(j)}.point(:,end)',End_Point);
                        %         node.f=close{end}.g+node.g+sum(param.length(3:5));
                            open{pos_in_open(j)}.path=param.path(1:num-3);
                            open{pos_in_open(j)}.R=param.R;
                            open{pos_in_open(j)}.r=param.r;%%[r_s,r_e]
                            open{pos_in_open(j)}.center=param.center;
                            open{pos_in_open(j)}.pos_id=param.pos_id;
                            open{pos_in_open(j)}.parent_id=param.parent_id;
                            open_f(pos_in_open(j))=open{pos_in_open(j)}.f;
                        else
                            display('ignore')
                        end
                    end
                %     display('修改');
                %     display(pos_in_open);
                else
                    node.point=param.point(:,1:num-2);
                    node.vtheta=param.vtheta(2:num-2-1);
                    node.vtheta_all=param.vtheta_all(1:num-3);
                
                %     node.Length=sum(param.length(1:2));
                
                    node.g=close{end}.g+sum(param.length(1:num-3));
                %     node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                    node.f=node.g+obtain_h(node.point(:,end)',End_Point);
                %         node.f=close{end}.g+node.g+sum(param.length(3:5));
                    node.path=param.path(1:num-3);
                    node.R=param.R;
                    node.r=param.r;%%[r_s,r_e]
                    node.center=param.center;
                    node.pos_id=param.pos_id;
                    node.parent_id=param.parent_id;
                    open_f=[open_f,node.f];
                    open{end+1}=node;
                end
            end
        end
    end
end


end
