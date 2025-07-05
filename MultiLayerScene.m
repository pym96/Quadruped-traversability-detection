% Generate a synthetic point cloud scene for a multi-layer 3D environment
clear all;
close all;

%% Scene Generation Parameters
% Define the scene dimensions
xRange = [0, 10];
yRange = [0, 10];
zRange = [0, 5];

% Grid resolution for point cloud generation
resolution = 0.1; % 10cm resolution

% Initialize point cloud array
nPoints = 0;
points = [];

% Generate the ground floor (z = 0 to 0.5m) with some obstacles
nGround = 5000;
groundX = xRange(1) + (xRange(2) - xRange(1)) * rand(nGround, 1);
groundY = yRange(1) + (yRange(2) - yRange(1)) * rand(nGround, 1);
groundZ = 0.2 * rand(nGround, 1); % Slight height variation
% Add an obstacle (e.g., a wall from x=3 to x=4, y=0 to 10, z=0 to 1)
obstacleIdx = groundX >= 3 & groundX <= 4;
groundZ(obstacleIdx) = 1 + 0.2 * rand(sum(obstacleIdx), 1); % Raise to 1m
points = [points; [groundX, groundY, groundZ]];
nPoints = nPoints + nGround;

% Generate a second floor (z = 2.5 to 3m) with a hole and stairs
nFloor2 = 3000;
floor2X = xRange(1) + (xRange(2) - xRange(1)) * rand(nFloor2, 1);
floor2Y = yRange(1) + (yRange(2) - yRange(1)) * rand(nFloor2, 1);
% Create a hole (e.g., x=6 to 8, y=6 to 8)
holeIdx = floor2X >= 6 & floor2X <= 8 & floor2Y >= 6 & floor2Y <= 8;
floor2Z = 2.7 + 0.2 * rand(nFloor2, 1);
floor2Z(holeIdx) = []; % Remove points in the hole
floor2X(holeIdx) = [];
floor2Y(holeIdx) = [];
points = [points; [floor2X, floor2Y, floor2Z]];
nPoints = nPoints + length(floor2X);

% Generate stairs connecting ground to second floor (e.g., x=0 to 2, y=0 to 1)
nStairs = 1000;
stairsX = linspace(0, 2, nStairs/2)';
stairsY = zeros(nStairs/2, 1) + 0.5 * rand(nStairs/2, 1);
stairsZ = linspace(0.2, 2.7, nStairs/2)';
points = [points; [stairsX, stairsY, stairsZ]];
nPoints = nPoints + nStairs/2;

% Add an overhang (e.g., a low ceiling at x=8 to 10, y=0 to 5, z=1 to 2)
nOverhang = 1500;
overhangX = 8 + 2 * rand(nOverhang, 1);
overhangY = 5 * rand(nOverhang, 1);
overhangZ = 1 + rand(nOverhang, 1);
points = [points; [overhangX, overhangY, overhangZ]];
nPoints = nPoints + nOverhang;

% Add noise to simulate real-world data
noise = 0.05 * randn(size(points)); % 5cm noise in x, y, z
points = points + noise;

% Ensure points stay within bounds
points = max(min(points, [xRange(2), yRange(2), zRange(2)]), [xRange(1), yRange(1), zRange(1)]);

% Create a pointCloud object for visualization
ptCloud = pointCloud(points);

% Add color information (optional - using height-based coloring)
colors = zeros(size(points, 1), 3);
normalizedHeight = (points(:,3) - min(points(:,3))) / (max(points(:,3)) - min(points(:,3)));
colors(:,1) = normalizedHeight;  % Red channel based on height
colors(:,2) = 0.5;              % Fixed green value
colors(:,3) = 1-normalizedHeight;% Blue channel inverse of height
ptCloud.Color = uint8(colors * 255);

% Visualize the raw point cloud
figure('Name', 'Raw Point Cloud');
pcshow(ptCloud);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Raw Point Cloud Scene');
grid on;
axis equal;
colormap jet;
c = colorbar;
c.Label.String = 'Height (m)';
caxis([min(points(:,3)), max(points(:,3))]);

%% Tomogram Processing
fprintf('\n=== Tomogram Processing ===\n');

% Initialize the tomogram processor with fine resolution
ds = 0.3;  % 30cm slice interval
rg = 0.1;  % 10cm grid resolution
processor = TomogramProcessor(ds, rg);

% Load the point cloud
processor.loadPointCloud(points);

% Process tomograms
fprintf('\nProcessing tomograms...\n');
processor.processTomograms();

% Visualize the processed tomograms
figure('Name', 'Processed Tomograms');
processor.visualizeSlices();
title('Processed Tomogram Analysis');

%% Path Planning Test Cases
fprintf('\n=== Path Planning Tests ===\n');

% Test Case 1: Ground floor navigation around obstacle
fprintf('\nTest Case 1: Ground floor navigation\n');
start1 = [0.5, 5.0, 0.2];  % Start before the wall
end1 = [5.0, 5.0, 0.2];    % End after the wall
path1 = processor.planPath(start1, end1);

% Test Case 2: Stair climbing
fprintf('\nTest Case 2: Stair climbing\n');
start2 = [0.5, 0.5, 0.2];  % Start at stairs bottom
end2 = [1.5, 0.5, 2.7];    % End at stairs top
path2 = processor.planPath(start2, end2);

% Test Case 3: Complex navigation (ground to second floor through hole)
fprintf('\nTest Case 3: Complex navigation\n');
start3 = [1.0, 1.0, 0.2];    % Start on ground floor
end3 = [7.0, 7.0, 2.7];      % End through the hole
path3 = processor.planPath(start3, end3);

%% Trajectory Optimization
fprintf('\n=== Trajectory Optimization ===\n');

% Optimize and visualize each path
if ~isempty(path1)
    fprintf('\nOptimizing ground floor path...\n');
    optimizedPath1 = processor.optimizePCTTrajectory(path1, processor.robotParams);
    processor.visualizeOptimizedTrajectory(path1, optimizedPath1);
end

if ~isempty(path2)
    fprintf('\nOptimizing stair climbing path...\n');
    optimizedPath2 = processor.optimizePCTTrajectory(path2, processor.robotParams);
    processor.visualizeOptimizedTrajectory(path2, optimizedPath2);
end

if ~isempty(path3)
    fprintf('\nOptimizing complex navigation path...\n');
    optimizedPath3 = processor.optimizePCTTrajectory(path3, processor.robotParams);
    processor.visualizeOptimizedTrajectory(path3, optimizedPath3);
end

% Save results
if ~exist('results', 'dir')
    mkdir('results');
end

save('results/multilayer_scene_results.mat', ...
    'points', 'path1', 'path2', 'path3', ...
    'optimizedPath1', 'optimizedPath2', 'optimizedPath3', ...
    'ds', 'rg');

fprintf('\nResults saved to results/multilayer_scene_results.mat\n');