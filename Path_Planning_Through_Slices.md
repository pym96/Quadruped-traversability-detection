To implement the path planning through slices as described in Section D of the paper "Efficient Global Navigational Planning in 3D Structures based on Point Cloud Tomography," I'll provide a MATLAB script that follows the A* modification approach using tomogram slices. The script will create a simplified 2.5D environment, plan paths across slices, and use gateway grids to connect different layers, as outlined in the text.

---

### MATLAB Implementation

The script below:
- Generates a synthetic point cloud to simulate a multi-layer environment.
- Creates tomogram slices with ground and ceiling elevations.
- Implements a modified A* algorithm to plan paths across slices, checking adjacent slices for gateway grids.
- Visualizes the path planning process.

```matlab
% Generate synthetic point cloud for a multi-layer environment
nPoints = 10000;
pointCloud = [rand(nPoints, 2) * 10, randi([0 5], nPoints, 1)]; % x, y in [0,10], z in [0,5]
% Add layers (e.g., simulate two floors with a height difference)
layer1 = pointCloud(pointCloud(:,3) <= 2.5, :);
layer2 = pointCloud(pointCloud(:,3) > 2.5, :) + [0 0 2.5]; % Second layer offset by 2.5m
pointCloud = [layer1; layer2];

% Discretize into tomogram slices (2.5D maps)
gridSize = 0.5;
xGrid = 0:gridSize:10;
yGrid = 0:gridSize:10;
zLayers = 0:2.5:5; % Two layers at z=0 and z=2.5
slices = cell(length(xGrid)-1, length(yGrid)-1, length(zLayers));
for k = 1:length(zLayers)
    for i = 1:length(xGrid)-1
        for j = 1:length(yGrid)-1
            idx = pointCloud(:,1) >= xGrid(i) & pointCloud(:,1) < xGrid(i+1) & ...
                  pointCloud(:,2) >= yGrid(j) & pointCloud(:,2) < yGrid(j+1) & ...
                  pointCloud(:,3) >= zLayers(k) & pointCloud(:,3) < zLayers(k) + 2.5;
            zValues = pointCloud(idx, 3);
            if ~isempty(zValues)
                slices{i,j,k} = [min(zValues), max(zValues)]; % [ground, ceiling]
            else
                slices{i,j,k} = [zLayers(k), zLayers(k) + 2.5]; % Default if no points
            end
        end
    end
end

% Cost map initialization (simplified interval cost c')
costMap = zeros(length(xGrid)-1, length(yGrid)-1, length(zLayers));
for k = 1:length(zLayers)
    for i = 1:length(xGrid)-1
        for j = 1:length(yGrid)-1
            [ground, ceiling] = slices{i,j,k};
            if ceiling - ground < 0.5 % Non-traversable if clearance < 0.5m
                costMap(i,j,k) = Inf;
            else
                costMap(i,j,k) = 1 / (ceiling - ground); % Inverse of clearance as cost
            end
        end
    end
end

% A* Path Planning with Slice Transitions
start = [1, 1, 1]; % [x_idx, y_idx, z_layer]
goal = [length(xGrid)-1, length(yGrid)-1, length(zLayers)]; % Top-right, top layer
openSet = {start};
cameFrom = containers.Map('KeyType', 'double', 'ValueType', 'any');
gScore = containers.Map('KeyType', 'double', 'ValueType', 'double');
gScore(num2str(start)) = 0;
fScore = containers.Map('KeyType', 'double', 'ValueType', 'double');
fScore(num2str(start)) = norm([goal(1:2) - start(1:2), 0]); % Heuristic: diagonal distance

while ~isempty(openSet)
    % Find node with minimum fScore
    [~, idx] = min(cellfun(@(x) fScore(num2str(x)), openSet));
    current = openSet{idx};
    if isequal(current, goal)
        break;
    end
    openSet(idx) = [];
    
    % Neighbors: 8-connected in current slice + adjacent slices
    i = current(1); j = current(2); k = current(3);
    neighbors = [];
    for di = -1:1
        for dj = -1:1
            ni = i + di; nj = j + dj;
            if ni > 0 && ni <= length(xGrid)-1 && nj > 0 && nj <= length(yGrid)-1
                % Same slice
                neighbors = [neighbors; [ni, nj, k]];
                % Adjacent slices if gateway exists
                if k > 1 && slices{ni,nj,k}(1) == slices{ni,nj,k-1}(1) && ...
                   costMap(ni,nj,k-1) < costMap(ni,nj,k)
                    neighbors = [neighbors; [ni, nj, k-1]]; % Gateway downward
                end
                if k < length(zLayers) && slices{ni,nj,k}(1) == slices{ni,nj,k+1}(1) && ...
                   costMap(ni,nj,k+1) < costMap(ni,nj,k)
                    neighbors = [neighbors; [ni, nj, k+1]]; % Gateway upward
                end
            end
        end
    end
    
    for next = neighbors'
        tentative_gScore = gScore(num2str(current)) + costMap(next(1), next(2), next(3)) + ...
                          norm([next(1:2) - current(1:2), 0]); % Euclidean + cost
        if ~isKey(gScore, num2str(next)) || tentative_gScore < gScore(num2str(next))
            cameFrom(num2str(next)) = current;
            gScore(num2str(next)) = tentative_gScore;
            fScore(num2str(next)) = tentative_gScore + norm([goal(1:2) - next(1:2), 0]);
            if ~ismember(next, openSet, 'rows')
                openSet{end+1} = next; %#ok<AGROW>
            end
        end
    end
end

% Reconstruct path
path = [];
current = goal;
while ~isequal(current, start)
    path = [current; path];
    current = cameFrom(num2str(current));
end
path = [start; path];

% Convert to world coordinates
pathWorld = zeros(size(path, 1), 3);
for i = 1:size(path, 1)
    pathWorld(i,:) = [(path(i,1)-1)*gridSize + gridSize/2, ...
                      (path(i,2)-1)*gridSize + gridSize/2, ...
                      slices{path(i,1), path(i,2), path(i,3)}(1)];
end

% Visualization
figure;
pcshow(pointCloud);
hold on;
plot3(pathWorld(:,1), pathWorld(:,2), pathWorld(:,3), 'r-', 'LineWidth', 2, 'Marker', '.');
for k = 1:length(zLayers)
    z = zLayers(k) + 0.5;
    plot([0 10], [0 0], z, 'b--', 'LineWidth', 1); % Ground plane
    plot([0 10], [10 10], z, 'b--', 'LineWidth', 1);
    plot([0 0], [0 10], z, 'b--', 'LineWidth', 1);
    plot([10 10], [0 10], z, 'b--', 'LineWidth', 1);
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Path Planning Through Slices');
grid on;
```

---

This script implements the key ideas from the paper:
- **Tomogram Slices**: The environment is divided into 2.5D slices based on z-layers, with ground and ceiling heights computed from the point cloud.
- **Modified A***: The algorithm searches through slices, checking 8-connected neighbors in the current slice and adjacent slices for gateway grids (where ground elevations match and costs are lower).
- **Gateway Grids**: Transitions between slices occur when the planner identifies nodes with lower costs in adjacent slices, mimicking the red- and blue-circled grids in Fig. 4.
- **Cost Function**: The interval cost \( c' \) is approximated as the inverse of clearance, with infinite cost for non-traversable regions.
- **Visualization**: The path is plotted in 3D, with slice boundaries shown as dashed lines.

You can enhance this by:
- Using real point cloud data (e.g., from a LIDAR scan).
- Adding more sophisticated cost functions based on terrain slope or robot dynamics.
- Optimizing the z-axis motion separately (as in the trajectory optimization section) to avoid overhangs.

Let me know if you'd like to refine any part of this!