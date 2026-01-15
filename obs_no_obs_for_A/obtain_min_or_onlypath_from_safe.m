function [min_path,path_safe_flag,flag_min]=obtain_min_or_onlypath_from_safe(type,param_all,flag_all)
%%type=1  =》选择最小，type=2=》onlypath    


%%先看安全不安全
if isstruct(flag_all)

        path_safe_flag_r=[];flag_min_r=0;    min_path_r=inf;
    for i = 1:length(flag_all.r(:,1))
        if all(flag_all.r(i, :) == 1)
            path_safe_flag_r=[path_safe_flag_r,i];%某行全1=>该路径安全
        end
    end
    path_safe_flag_R=[];flag_min_R=0;    min_path_R=inf;
    for i = 1:length(flag_all.R(:,1))
        if all(flag_all.R(i, :) == 1)
            path_safe_flag_R=[path_safe_flag_R,i];%某行全1=>该路径安全
        end
    end

    flag_min.R=flag_min_R;
    flag_min.r=flag_min_r;
    min_path.R=min_path_R;
    min_path.r=min_path_r;
    path_safe_flag.R=path_safe_flag_R;
    path_safe_flag.r=path_safe_flag_r;




else

    %%如果有安全的
    path_safe_flag=[];    min_path=inf;flag_min=0;
    for i = 1:length(flag_all(:,1))
        if all(flag_all(i, :) == 1)
            path_safe_flag=[path_safe_flag,i];%某行全1=>该路径安全
        end
    end
%     display(path_safe_flag)


end







if type==1%%选择最短=>结束程序
    if isstruct(flag_all)
         %%选择安全中最短的
    
        if ~isempty(path_safe_flag_r)
            for i=1:length(path_safe_flag_r)
                if param_all(path_safe_flag_r(i)).Length<min_path_r
                    min_path_r=param_all(path_safe_flag_r(i)).Length;
                    flag_min_r=i;
                end
            end
        end
    
    
        if ~isempty(path_safe_flag_R)
            for i=1:length(path_safe_flag_R)
                if param_all(path_safe_flag_r(i)).Length<min_path_R
                    min_path_R=param_all(path_safe_flag_r(i)).Length;
                    flag_min_R=i;
                end
            end
        end

    flag_min.R=flag_min_R;
    flag_min.r=flag_min_r;
    min_path.R=min_path_R;
    min_path.r=min_path_r;
    path_safe_flag.R=path_safe_flag_R;
    path_safe_flag.r=path_safe_flag_r;
    

    else
                %%选择安全中最短的
        [rowCount, colCount] = size(param_all);            %%%%%[3,4]
        range_all=[length(param_all{1,1})];
        for k=1:(colCount-1)                    
            range_all=[range_all,range_all(end)+length(param_all{1,k+1})];
        end
            %%%%[3,7]

        if ~isempty(path_safe_flag)            
            for i=1:length(path_safe_flag)
                num=path_safe_flag(i);
                index=find(num > range_all, 1, 'last');                
                %%%%判断属于哪个部分
                if isempty(index)
%                     display([param_all{1}(num).Length,min_path])
                    if param_all{1}(num).Length<min_path
                        min_path=param_all{1}(num).Length;
                        flag_min=i;
                    end

                else
                        
                    if param_all{1,index+1}(num-range_all(index)).Length<min_path
                        min_path=param_all{1,index+1}(num-range_all(index)).Length;
                        flag_min=i;
                    end

                end
            end
        end
    end

end




% if isstruct(flag_all)
% 
%         path_safe_flag_r=[];flag_min_r=0;    min_path_r=inf;
%     for i = 1:length(flag_all.r(:,1))
%         if all(flag_all.r(i, :) == 1)
%             path_safe_flag_r=[path_safe_flag_r,i];%某行全1=>该路径安全
%         end
%     end
%     path_safe_flag_R=[];flag_min_R=0;    min_path_R=inf;
%     for i = 1:length(flag_all.R(:,1))
%         if all(flag_all.R(i, :) == 1)
%             path_safe_flag_R=[path_safe_flag_R,i];%某行全1=>该路径安全
%         end
%     end
% 
%     %%选择安全中最短的
% 
%     if ~isempty(path_safe_flag_r)
%         for i=1:length(path_safe_flag_r)
%             if param_all(path_safe_flag_r(i)).Length<min_path_r
%                 min_path_r=param_all(path_safe_flag_r(i)).Length;
%                 flag_min_r=i;
%             end
%         end
%     end
% 
% 
%     if ~isempty(path_safe_flag_R)
%         for i=1:length(path_safe_flag_R)
%             if param_all(path_safe_flag_r(i)).Length<min_path_R
%                 min_path_R=param_all(path_safe_flag_r(i)).Length;
%                 flag_min_R=i;
%             end
%         end
%     end
% 
%     flag_min.R=flag_min_R;
%     flag_min.r=flag_min_r;
%     min_path.R=min_path_R;
%     min_path.r=min_path_r;
%     path_safe_flag.R=path_safe_flag_R;
%     path_safe_flag.r=path_safe_flag_r;
% 
% 
% 
% else
% 
%     %%如果有安全的
%     path_safe_flag=[];
%     for i = 1:length(flag_all(:,1))
%         if all(flag_all(i, :) == 1)
%             path_safe_flag=[path_safe_flag,i];%某行全1=>该路径安全
%         end
%     end
%     min_path=inf;
%     flag_min=0;
%     %%选择安全中最短的
%     if ~isempty(path_safe_flag)
%         for i=1:length(path_safe_flag)
%             if param_all(path_safe_flag(i)).Length<min_path
%                 min_path=param_all(path_safe_flag(i)).Length;
%                 flag_min=i;
%             end
%         end
%     end

end