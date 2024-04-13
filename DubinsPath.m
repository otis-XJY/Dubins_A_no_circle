function path = DubinsPath(param,Stepsize)
switch param.type
    case 1
        path = PathfsRSR(param,Stepsize);
    case 2
        path = PathfsRSL(param,Stepsize);
    case 3
        path = PathfsLSL(param,Stepsize);
    case 4
        path = PathfsLSR(param,Stepsize);
    case 5
        path = PathRSR(param,Stepsize);
    case 6
        path = PathRSL(param,Stepsize);
    case 7
        path = PathLSL(param,Stepsize);
    case 8
        path = PathLSR(param,Stepsize);
    otherwise
        warning('No type')
end

% display(path)
end

function path_RSR = PathfsRSR(paramN,Stepsize)
    step = zeros(3,1);
    for i = 1:3
        step(i) = 1/Stepsize*paramN.length(i);
    end
    %直线段
    Line1x = linspace(paramN.point(1,2),paramN.point(1,3),step(2));
    Line1y = linspace(paramN.point(2,2),paramN.point(2,3),step(2));
    %弧
    C1 = linspace(0,paramN.phy(1),step(1)); 
    C2 = linspace(0,paramN.phy(2),step(3));
    
    Round1x = paramN.center(1,1) + paramN.r(1)*cos(paramN.theta(1) - C1);
    Round1y = paramN.center(2,1) + paramN.r(1)*sin(paramN.theta(1) - C1);
    Round2x = paramN.center(1,2) + paramN.r(2)*cos(paramN.theta(3) - C2);
    Round2y = paramN.center(2,2) + paramN.r(2)*sin(paramN.theta(3) - C2);
    
    path_RSR{1}=[Round1x;Round1y];
    path_RSR{2}=[Line1x;Line1y];
    path_RSR{3}=[Round2x;Round2y];

%      path_RSR = [Round1x; Line1x; Round2x;...
%                  Round1y; Line1y; Round2y];

%      path_RSR = [Round1x, Line1x, Round2x;...
%                  Round1y, Line1y, Round2y];
    
end

function path_RSL = PathfsRSL(paramN,Stepsize)
    step = zeros(3,1);
    for i = 1:3
        step(i) = 1/Stepsize*paramN.length(i);
    end
    %直线段
    Line1x = linspace(paramN.point(1,2),paramN.point(1,3),step(2));
    Line1y = linspace(paramN.point(2,2),paramN.point(2,3),step(2));
    %弧
    C1 = linspace(0,paramN.phy(1),step(1)); 
    C2 = linspace(0,paramN.phy(2),step(3));
    
    Round1x = paramN.center(1,1) + paramN.r(1)*cos(paramN.theta(1) - C1);
    Round1y = paramN.center(2,1) + paramN.r(1)*sin(paramN.theta(1) - C1);
    Round2x = paramN.center(1,2) + paramN.r(2)*cos(paramN.theta(3) + C2);
    Round2y = paramN.center(2,2) + paramN.r(2)*sin(paramN.theta(3) + C2);
    
    path_RSL{1}=[Round1x;Round1y];
    path_RSL{2}=[Line1x;Line1y];
    path_RSL{3}=[Round2x;Round2y];

%      path_RSL = [Round1x; Line1x; Round2x;...
%                  Round1y; Line1y; Round2y];

%      path_RSL = [Round1x, Line1x, Round2x;...
%                  Round1y, Line1y, Round2y];
    
end

function path_LSL = PathfsLSL(paramN,Stepsize)
    step = zeros(3,1);
    for i = 1:3
        step(i) = 1/Stepsize*paramN.length(i);
    end
    %画直线段
    Line1x = linspace(paramN.point(1,2),paramN.point(1,3),step(2));
    Line1y = linspace(paramN.point(2,2),paramN.point(2,3),step(2));
    %画弧
    C1 = linspace(0,paramN.phy(1),step(1)); 
    C2 = linspace(0,paramN.phy(2),step(3));
    
    Round1x = paramN.center(1,1) + paramN.r(1)*cos(paramN.theta(1) + C1);
    Round1y = paramN.center(2,1) + paramN.r(1)*sin(paramN.theta(1) + C1);
    Round2x = paramN.center(1,2) + paramN.r(2)*cos(paramN.theta(3) + C2);
    Round2y = paramN.center(2,2) + paramN.r(2)*sin(paramN.theta(3) + C2);
    
    path_LSL{1}=[Round1x;Round1y];
    path_LSL{2}=[Line1x;Line1y];
    path_LSL{3}=[Round2x;Round2y];

%      path_LSL = [Round1x; Line1x; Round2x;...
%                  Round1y; Line1y; Round2y];

%      path_LSL = [Round1x, Line1x, Round2x;...
%                  Round1y, Line1y, Round2y];
    
end

function path_LSR = PathfsLSR(paramN,Stepsize)
    step = zeros(3,1);
    for i = 1:3
        step(i) = 1/Stepsize*paramN.length(i);
    end
    %直线段
    Line1x = linspace(paramN.point(1,2),paramN.point(1,3),step(2));
    Line1y = linspace(paramN.point(2,2),paramN.point(2,3),step(2));
    %弧
    C1 = linspace(0,paramN.phy(1),step(1)); 
    C2 = linspace(0,paramN.phy(2),step(3));
    
    Round1x = paramN.center(1,1) + paramN.r(1)*cos(paramN.theta(1) + C1);
    Round1y = paramN.center(2,1) + paramN.r(1)*sin(paramN.theta(1) + C1);
    Round2x = paramN.center(1,2) + paramN.r(2)*cos(paramN.theta(3) - C2);
    Round2y = paramN.center(2,2) + paramN.r(2)*sin(paramN.theta(3) - C2);
    
    path_LSR{1}=[Round1x;Round1y];
    path_LSR{2}=[Line1x;Line1y];
    path_LSR{3}=[Round2x;Round2y];

%      path_LSR = [Round1x; Line1x; Round2x;...
%                  Round1y; Line1y; Round2y];
end

function path_RSR = PathRSR(param,Stepsize)
    step = zeros(5,1);
    for i = 1:5
        step(i) = 1/Stepsize*param.length(i);
    end
    %直线段
    Line1x = linspace(param.point(1,2),param.point(1,3),step(2));
    Line1y = linspace(param.point(2,2),param.point(2,3),step(2));
    Line2x = linspace(param.point(1,4),param.point(1,5),step(4));
    Line2y = linspace(param.point(2,4),param.point(2,5),step(4));
    %弧
    C1 = linspace(0,param.phy(1),step(1)); 
    C2 = linspace(0,param.phy(2),step(3));
    C3 = linspace(0,param.phy(3),step(5));
    
    Round1x = param.center(1,1) + param.r(1)*cos(param.theta(1) - C1);
    Round1y = param.center(2,1) + param.r(1)*sin(param.theta(1) - C1);
    Round2x = param.center(1,2) + param.R*cos(param.theta(3) - C2);
    Round2y = param.center(2,2) + param.R*sin(param.theta(3) - C2);
    Round3x = param.center(1,3) + param.r(2)*cos(param.theta(5) - C3);
    Round3y = param.center(2,3) + param.r(2)*sin(param.theta(5) - C3);

    path_RSR{1}=[Round1x;Round1y];
    path_RSR{2}=[Line1x;Line1y];
    path_RSR{3}=[Round2x;Round2y];
    path_RSR{4}=[Line2x;Line2y];
    path_RSR{5}=[Round3x;Round3y];

%      path_RSR = [Round1x; Line1x; Round2x; Line2x; Round3x;...
%                  Round1y; Line1y; Round2y; Line2y; Round3y];
    
%      path_RSR = [Round1x, Line1x, Round2x, Line2x, Round3x;...
%                  Round1y, Line1y, Round2y, Line2y, Round3y];
end

function path_RSL = PathRSL(param,Stepsize)
    step = zeros(5,1);
    for i = 1:5
        step(i) = 1/Stepsize*param.length(i);
    end
    %直线段
    Line1x = linspace(param.point(1,2),param.point(1,3),step(2));
    Line1y = linspace(param.point(2,2),param.point(2,3),step(2));
    Line2x = linspace(param.point(1,4),param.point(1,5),step(4));
    Line2y = linspace(param.point(2,4),param.point(2,5),step(4));
    %弧
    C1 = linspace(0,param.phy(1),step(1)); 
    C2 = linspace(0,param.phy(2),step(3));
    C3 = linspace(0,param.phy(3),step(5));
    
    Round1x = param.center(1,1) + param.r(1)*cos(param.theta(1) - C1);
    Round1y = param.center(2,1) + param.r(1)*sin(param.theta(1) - C1);
    Round2x = param.center(1,2) + param.R*cos(param.theta(3) + C2);
    Round2y = param.center(2,2) + param.R*sin(param.theta(3) + C2);
    Round3x = param.center(1,3) + param.r(2)*cos(param.theta(5) + C3);
    Round3y = param.center(2,3) + param.r(2)*sin(param.theta(5) + C3);
 

    path_RSL{1}=[Round1x;Round1y];
    path_RSL{2}=[Line1x;Line1y];
    path_RSL{3}=[Round2x;Round2y];
    path_RSL{4}=[Line2x;Line2y];
    path_RSL{5}=[Round3x;Round3y];

%      path_RSL = [Round1x; Line1x; Round2x; Line2x; Round3x;...
%                  Round1y; Line1y; Round2y; Line2y; Round3y];

%      path_RSLx = [Round1x, Line1x, Round2x, Line2x, Round3x];
%      path_RSLy = [Round1y, Line1y, Round2y, Line2y, Round3y];
%      path_RSL = [path_RSLx; path_RSLy];
    
end

function path_LSL = PathLSL(param,Stepsize)
    step = zeros(5,1);
    for i = 1:5
        step(i) = 1/Stepsize*param.length(i);
    end
    Line1x = linspace(param.point(1,2),param.point(1,3),step(2));
    Line1y = linspace(param.point(2,2),param.point(2,3),step(2));
    Line2x = linspace(param.point(1,4),param.point(1,5),step(4));
    Line2y = linspace(param.point(2,4),param.point(2,5),step(4));
    %弧
    C1 = linspace(0,param.phy(1),step(1)); 
    C2 = linspace(0,param.phy(2),step(3));
    C3 = linspace(0,param.phy(3),step(5));
    
    Round1x = param.center(1,1) + param.r(1)*cos(param.theta(1) + C1);
    Round1y = param.center(2,1) + param.r(1)*sin(param.theta(1) + C1);
    Round2x = param.center(1,2) + param.R*cos(param.theta(3) + C2);
    Round2y = param.center(2,2) + param.R*sin(param.theta(3) + C2);
    Round3x = param.center(1,3) + param.r(2)*cos(param.theta(5) + C3);
    Round3y = param.center(2,3) + param.r(2)*sin(param.theta(5) + C3);
    
    path_LSL{1}=[Round1x;Round1y];
    path_LSL{2}=[Line1x;Line1y];
    path_LSL{3}=[Round2x;Round2y];
    path_LSL{4}=[Line2x;Line2y];
    path_LSL{5}=[Round3x;Round3y];
% %      path_LSL = [Round1x, Line1x, Round2x, Line2x, Round3x;...
% %                  Round1y, Line1y, Round2y, Line2y, Round3y];
%      path_LSL = [Round1x; Line1x; Round2x; Line2x; Round3x;...
%                  Round1y; Line1y; Round2y; Line2y; Round3y];
end

function path_LSR = PathLSR(param,Stepsize)
    step = zeros(5,1);
    for i = 1:5
        step(i) = 1/Stepsize*param.length(i);
    end
    %直线段
    Line1x = linspace(param.point(1,2),param.point(1,3),step(2));
    Line1y = linspace(param.point(2,2),param.point(2,3),step(2));
    Line2x = linspace(param.point(1,4),param.point(1,5),step(4));
    Line2y = linspace(param.point(2,4),param.point(2,5),step(4));
    %弧
    C1 = linspace(0,param.phy(1),step(1)); 
    C2 = linspace(0,param.phy(2),step(3));
    C3 = linspace(0,param.phy(3),step(5));
    
    Round1x = param.center(1,1) + param.r(1)*cos(param.theta(1) + C1);
    Round1y = param.center(2,1) + param.r(1)*sin(param.theta(1) + C1);
    Round2x = param.center(1,2) + param.R*cos(param.theta(3) - C2);
    Round2y = param.center(2,2) + param.R*sin(param.theta(3) - C2);
    Round3x = param.center(1,3) + param.r(2)*cos(param.theta(5) - C3);
    Round3y = param.center(2,3) + param.r(2)*sin(param.theta(5) - C3);
    
    path_LSR{1}=[Round1x;Round1y];
    path_LSR{2}=[Line1x;Line1y];
    path_LSR{3}=[Round2x;Round2y];
    path_LSR{4}=[Line2x;Line2y];
    path_LSR{5}=[Round3x;Round3y];
%      path_LSR = [Round1x; Line1x; Round2x; Line2x; Round3x;...
%                  Round1y; Line1y; Round2y; Line2y; Round3y];
    
end