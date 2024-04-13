function param_all_new=obtian_short_param(param_all)

min_length=[];

for i=1:length(param_all)
    for j=1:length(param_all{i})
        if isempty(param_all{i}(j).Length)
            param_all{i}(j).Length=inf;
        end
            min_length=[min_length,param_all{i}(j).Length];
    end


    [sorted_A, index] = sort(min_length);
%     min_two = sorted_A;
    min_two_index = index(1:(length(find(min_length~=inf))/2));

    for k=1:length(min_two_index)
        param_all_new{i}(k)=param_all{i}(min_two_index(k));
    end
    min_length=[];
end

    

    
