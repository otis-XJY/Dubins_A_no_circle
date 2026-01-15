function myFunction(varargin)
    disp('接收到的参数个数：');
    disp(length(varargin));
    disp('参数列表：');
    disp(varargin{1});
end
