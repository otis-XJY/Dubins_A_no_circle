% 生成一些示例数据
x = linspace(0, 10, 100);
y = sin(x);

% 绘制图形
plot(x, y);
hold on;

% 创建文本框显示坐标和圆半径
txt = text(0, 0, '', 'Color', 'red');
txtRadius = text(0, 0, '', 'Color', 'blue');

% 初始化选点位置和圆半径
selectedPoints = [];
radius = 1;

% 设置鼠标移动和点击回调函数
set(gcf, 'WindowButtonMotionFcn', @(src, event) mouseMoveCallback(src, event, txt));
set(gcf, 'WindowButtonDownFcn', @(src, event) mouseClickCallback(src, event, txt, selectedPoints,txtRadius));
storedPoints = get(gcf, 'UserData');
function mouseMoveCallback(src, event, txt)
    % 获取当前坐标
    pt = get(gca, 'CurrentPoint');
    x = pt(1, 1);
    y = pt(1, 2);
    
    % 更新文本框显示坐标
    set(txt, 'String', ['当前鼠标位置：(', num2str(x), ', ', num2str(y), ')']);
    set(txt, 'Position', [x, y]);
end

function mouseClickCallback(src, event, txt, selectedPoints,txtRadius)
    % 获取当前点击类型
    selectionType = get(gcf, 'SelectionType');
    
    % 判断点击事件
    if strcmp(selectionType, 'normal')  % 左键点击
        % 获取点击位置
        pt = get(gca, 'CurrentPoint');
        x = pt(1, 1);
        y = pt(1, 2);
        
        % 更新选点位置
        selectedPoints = [selectedPoints; x, y];
        set(gcf, 'UserData', selectedPoints);
        % 显示选点坐标
        disp(['已选点：(', num2str(x), ', ', num2str(y), ')']);
        
        % 绘制圆形
        
    elseif strcmp(selectionType, 'alt')  % 右键点击
        % 退出选点
        disp('退出选点。');
        set(gcf, 'WindowButtonMotionFcn', '');
        set(gcf, 'WindowButtonDownFcn', '');
        
    elseif strcmp(selectionType, 'extend')
        disp('fuck')
    end
    % 从命令行获取圆半径
    prompt = '请输入圆的半径大小：';
    radius = input(prompt);
    
    % 更新文本框显示圆半径
    set(txtRadius, 'String', ['当前圆半径：', num2str(radius)]);
    set(txtRadius, 'Position', [0, 0]);
    
    plot(x, y, 'ro', 'MarkerSize', radius);  % 在点上绘制红色圆形
end

