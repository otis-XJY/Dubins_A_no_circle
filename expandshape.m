function expandedPolygon = expandshape(polygon, radius)
    numVertices = size(polygon, 1);
    expandedPolygon=[];
%     expandedPolygon = zeros(numVertices, 2);
    
    for i = 1:(numVertices-1)
        % Get current and next vertices
        currentVertex = polygon(i, :);
        nextVertex = polygon(mod(i, numVertices) + 1, :);
        
        % Compute edge vector
        edgeVector = nextVertex - currentVertex;
        
        % Compute edge normal vector (rotate by 90 degrees)
        edgeNormal = -[-edgeVector(2), edgeVector(1)];
        edgeNormal = edgeNormal / norm(edgeNormal);
        
        % Compute expanded vertex position
        expandedVertex1 = currentVertex + radius * edgeNormal;
        expandedVertex2 = nextVertex + radius * edgeNormal;
        % Store expanded vertex
        expandedPolygon(end+1, :) = expandedVertex1;
        expandedPolygon(end+1, :) = expandedVertex2;
    end
    expandedPolygon=[expandedPolygon;expandedPolygon(1,:)];
    % Plot circles at original vertices
%     hold on;
%     plot(polygon(:, 1), polygon(:, 2), 'b.-'); % Plot original polygon
%     plot(expandedPolygon(:, 1), expandedPolygon(:, 2), 'r.-'); % Plot expanded polygon
    
    % Plot circles at original vertices
%     theta = linspace(0, 2 * pi, 100);
%     for i = 1:numVertices
%         center = polygon(i, :);
%         circleX = center(1) + radius * cos(theta);
%         circleY = center(2) + radius * sin(theta);
%         plot(circleX, circleY, 'g-');
%     end
    
%     axis equal;
%     hold off;
end
