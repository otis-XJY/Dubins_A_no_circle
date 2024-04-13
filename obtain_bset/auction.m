% 拍卖算法
% 输入;第一个是边赋权完全偶图，即输入每一个agent对于每一个target的价值，
%      第二个是用于确定收敛精度的参数（可以不输入）
% 输出：最优分配assignments和最小代价total_benefit，其中最assignments
%       第一行（1，2，3，4...）的值代表agent的编号，第二行的值代表target
%       的编号若第二行第三个元素的值为4则表示将第四个target分配给第三个agent

%拍卖算法
%输入: --benefits (i, j)为各agent (受控机)处理每个target (目标)的价值(效率)，
%      是一个边赋权完全偶图，其中
%      i:agent编号
%      j:target编号
%     --epsilon是用于确定收敛精度的参数(可以缺省，默认值为0.1*1 / (agent的数量+1))
%
%输出:--assignments为最优分配结果,assignments(1,i)表示将第i个agent分配给
%       第assignments(2,i)个target
%     --total_ benefit 为整体最小代价，其值表示所有agent对于所有agent的分配中,
%       代价值最小的一种分配



function [assignments, total_benefit,benefit_count] = auction(benefits, epsilon)

a=(1+max(max(benefits)));
benefits=a-benefits;

%这一段是将任意M*N矩阵转化成方阵
[row,column]=size(benefits);
MIN = min(benefits,[],'all');
MAX = max(benefits,[],'all');
if(row>column)
    counter = column;
    X=zeros(row,row-column);
    X=MIN-X;
    benefits=[benefits X];
elseif(row<column)
    counter = row;
    X=zeros(column-row,column);
    X=MAX-X+20;
    benefits=[benefits;X];
end


N = length(benefits);%N=benefits中最大维度的元素数量
benefits_test = benefits;

people = [linspace(1,N,N); zeros(1,N)]; % 第一行的人，第二行表示他们是否满意
object_prices = zeros(1,N); % 物品的price
assignments = [linspace(1,N,N); linspace(1,N,N)]; % 第一排人，第二排物品

if(nargin < 2)%nargin输入参数数目
    epsilon = 1 / 100*(N+1);%改变收敛速度和精确程度,越小精度越高，收敛时间越长
end

count = 1;%用于记录total_benefit的变化
while sum(people(2,:)) < N%有人不满意这个分配，则进入循环
    for i = 1:N
        if(people(2,i) == 0)%有人不满意这个分配，更新物品的price
            options = benefits(i,:) - object_prices(1,:);%更新当前target的价格
            [best_option, index] = max(options);
            
            if(best_option > 0)
                j = 1;
                while (index ~= assignments(2,j))
                    j = j+1;
                end
                
                temp = assignments(2,j);%将当前最高价target分配给出价人j
                assignments(2,j) = assignments(2,i);
                assignments(2,i) = temp;
                
                options(index) = -1;%进入下一轮竞价
                next_best_option = max(options);
                
                if(next_best_option > 0)%下一轮的出价
                    bid = best_option - next_best_option;
                else
                    bid = best_option;
                end
                
                object_prices(index) = object_prices(index) + bid + epsilon;
                
            else
                people(2,i) = 1;%第i个人同意分配，并将他对于其他target的报价改为0，即不参与后续竞价
                benefits(:,assignments(2,i)) = zeros(1,N);
            end
            benefit_temp = 0;
            for p = 1:counter
                j = assignments(2,p);
                benefit_temp = benefit_temp + benefits_test(p,j);%计算总收益
            end            
            benefit_temp = a*counter-benefit_temp;
            benefit_count(count) = benefit_temp;
            count = count+1;
        end
    end
end
total_benefit = benefit_temp;
if(row>column)
for i=1:row
   if (assignments(2,i)>column)
      assignments(2,i) = 0;
   end
end
elseif(row<column)
     assignments(:,row+1:column) = [];
end
% total_benefit = 0;
% for i = 1:counter
%     j = assignments(2,i);
%     total_benefit = total_benefit + benefits_test(i,j);%计算总收益
% end
% total_benefit=a*counter-total_benefit;
end