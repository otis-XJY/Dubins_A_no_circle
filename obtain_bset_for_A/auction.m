% �����㷨
% ����;��һ���Ǳ߸�Ȩ��ȫżͼ��������ÿһ��agent����ÿһ��target�ļ�ֵ��
%      �ڶ���������ȷ���������ȵĲ��������Բ����룩
% ��������ŷ���assignments����С����total_benefit��������assignments
%       ��һ�У�1��2��3��4...����ֵ����agent�ı�ţ��ڶ��е�ֵ����target
%       �ı�����ڶ��е�����Ԫ�ص�ֵΪ4���ʾ�����ĸ�target�����������agent

%�����㷨
%����: --benefits (i, j)Ϊ��agent (�ܿػ�)����ÿ��target (Ŀ��)�ļ�ֵ(Ч��)��
%      ��һ���߸�Ȩ��ȫżͼ������
%      i:agent���
%      j:target���
%     --epsilon������ȷ���������ȵĲ���(����ȱʡ��Ĭ��ֵΪ0.1*1 / (agent������+1))
%
%���:--assignmentsΪ���ŷ�����,assignments(1,i)��ʾ����i��agent�����
%       ��assignments(2,i)��target
%     --total_ benefit Ϊ������С���ۣ���ֵ��ʾ����agent��������agent�ķ�����,
%       ����ֵ��С��һ�ַ���



function [assignments, total_benefit,benefit_count] = auction(benefits, epsilon)

a=(1+max(max(benefits)));
benefits=a-benefits;

%��һ���ǽ�����M*N����ת���ɷ���
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


N = length(benefits);%N=benefits�����ά�ȵ�Ԫ������
benefits_test = benefits;

people = [linspace(1,N,N); zeros(1,N)]; % ��һ�е��ˣ��ڶ��б�ʾ�����Ƿ�����
object_prices = zeros(1,N); % ��Ʒ��price
assignments = [linspace(1,N,N); linspace(1,N,N)]; % ��һ���ˣ��ڶ�����Ʒ

if(nargin < 2)%nargin���������Ŀ
    epsilon = 1 / 100*(N+1);%�ı������ٶȺ;�ȷ�̶�,ԽС����Խ�ߣ�����ʱ��Խ��
end

count = 1;%���ڼ�¼total_benefit�ı仯
while sum(people(2,:)) < N%���˲�����������䣬�����ѭ��
    for i = 1:N
        if(people(2,i) == 0)%���˲�����������䣬������Ʒ��price
            options = benefits(i,:) - object_prices(1,:);%���µ�ǰtarget�ļ۸�
            [best_option, index] = max(options);
            
            if(best_option > 0)
                j = 1;
                while (index ~= assignments(2,j))
                    j = j+1;
                end
                
                temp = assignments(2,j);%����ǰ��߼�target�����������j
                assignments(2,j) = assignments(2,i);
                assignments(2,i) = temp;
                
                options(index) = -1;%������һ�־���
                next_best_option = max(options);
                
                if(next_best_option > 0)%��һ�ֵĳ���
                    bid = best_option - next_best_option;
                else
                    bid = best_option;
                end
                
                object_prices(index) = object_prices(index) + bid + epsilon;
                
            else
                people(2,i) = 1;%��i����ͬ����䣬��������������target�ı��۸�Ϊ0�����������������
                benefits(:,assignments(2,i)) = zeros(1,N);
            end
            benefit_temp = 0;
            for p = 1:counter
                j = assignments(2,p);
                benefit_temp = benefit_temp + benefits_test(p,j);%����������
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
%     total_benefit = total_benefit + benefits_test(i,j);%����������
% end
% total_benefit=a*counter-total_benefit;
end