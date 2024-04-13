function param_safe=obtain_safeparam(path_safe_flag,param_all)
if ~isempty(param_all)
    [rowCount, colCount] = size(param_all);    %%%%%[3,4,3] 
    range_all=[length(param_all{1,1})];
    for k=1:(colCount-1)                    
        range_all=[range_all,range_all(end)+length(param_all{1,k+1})];%%%%[3,7,10]
    end
    
    
    if ~isempty(path_safe_flag)
        for i=1:length(path_safe_flag)
            num=path_safe_flag(i);
            index=find(num > range_all, 1, 'last');
            
            %%%%判断属于哪个部分
            if isempty(index)
        
                param_safe{1,1}(num)=param_all{1,1}(num);
            else
        
                param_safe{1,index+1}(num-range_all(index))=param_all{1,index+1}(num-range_all(index));
        
            end
        end
    else
            param_safe={};

    end
else
    param_safe={};

end