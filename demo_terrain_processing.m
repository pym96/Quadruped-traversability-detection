% Demo script for terrain point cloud processing
% This script demonstrates how to use the TerrainPointCloudProcessor class
% for analyzing terrain data for quadruped navigation

%% Generate sample point cloud data (replace this with your actual point cloud data)
% Create a sample terrain with stairs
[X, Y] = meshgrid(-2:0.02:2, -2:0.02:2);
Z = zeros(size(X));

% Add some terrain features
% Base terrain with slight slope
Z = Z + 0.1 * X + 0.05 * Y;

% Add stairs
stair_height = 0.2;
stair_width = 0.4;
for i = 1:3
    mask = (X >= (i-1)*stair_width) & (X < i*stair_width) & (Y >= -1) & (Y <= 1);
    Z(mask) = Z(mask) + i*stair_height;
end

% Convert to point cloud format
points = [X(:), Y(:), Z(:)];

%% Process the point cloud
% Create processor instance with 5cm grid size
processor = TerrainPointCloudProcessor(0.05);

% Load and process point cloud
processor = processor.loadPointCloud(points);
processor = processor.processPointCloud();

% Visualize results
processor.visualizeTerrain();

% The processed data can now be used for path planning:
% - processor.heightMap: Contains terrain elevation data
% - processor.normalMap: Contains surface normal vectors
% - processor.traversabilityMap: Binary map of traversable regions

% Display some statistics
fprintf('Terrain Statistics:\n');
fprintf('Number of points: %d\n', size(points, 1));
fprintf('Terrain dimensions: %.2fm x %.2fm\n', ...
    max(X(:)) - min(X(:)), max(Y(:)) - min(Y(:)));
fprintf('Maximum height: %.2fm\n', max(Z(:)));
fprintf('Minimum height: %.2fm\n', min(Z(:)));

% Calculate percentage of traversable terrain
traversable_percentage = 100 * sum(processor.traversabilityMap(:)) / numel(processor.traversabilityMap);
fprintf('Traversable terrain: %.1f%%\n', traversable_percentage); 