%% Analyze stair traversability
% This script combines the simple stair scene with our traversability analysis

clear; clc;

% First generate the stair scene
run('simple_stair_perception.m');

% Initialize the tomogram processor with fine resolution
ds = 0.2;  % 50cm slice interval
rg = 0.1;  % 10cm grid resolution
processor = TomogramProcessor(ds, rg);

% Load the point cloud
processor.loadPointCloud(pcd.Location);

% Process tomograms
fprintf('\n=== Step 1: Processing Tomograms ===\n');
processor.processTomograms();

% Visualize the raw tomogram slices
figure('Name', 'Raw Tomogram Slices');
processor.visualizeSlices();
title('Raw Tomogram Slices');

% Simplify tomograms by removing redundant slices
processor.simplifyTomograms();

% Visualize the simplified tomogram slices
figure('Name', 'Simplified Tomogram Slices');
processor.visualizeSlices();
title('Simplified Tomogram Slices');

% Add title with scene parameters
sgtitle(sprintf('Stair Scene Analysis\nSteps: %d up & down, Height: %.2fm, Depth: %.2fm', ...
    N, STEP_H, STEP_D));

% Print scene statistics
fprintf('\nScene Statistics:\n');
fprintf('Number of steps: %d (up and down)\n', N);
fprintf('Step dimensions:\n');
fprintf('  Width: %.2f m\n', STEP_W);
fprintf('  Height: %.2f m\n', STEP_H);
fprintf('  Depth: %.2f m\n', STEP_D);
fprintf('Total points: %d\n', size(pcd.Location, 1));
fprintf('Noise level: %.3f m\n', NOISE);

% Path planning parameters
fprintf('\n=== Step 2: Path Planning ===\n');
% Use the algorithm-found traversable positions for better success
startPos = [0.2, 0.08, 0.10];  % Start: algorithm-found traversable position
endPos = [0.2, 4.2, 0.10];     % End: latest algorithm-found traversable position

fprintf('Planning path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]\n', ...
    startPos(1), startPos(2), startPos(3), endPos(1), endPos(2), endPos(3));

% Plan initial path
rawPath = processor.planPath(startPos, endPos);

if isempty(rawPath)
    fprintf('Initial path planning failed. Running detailed analysis...\n');
    processor.analyzeStairScenario(startPos, endPos);
    error('Failed to find a valid path!');
end

fprintf('Initial path found with %d waypoints\n', size(rawPath, 1));

% Trajectory optimization
fprintf('\n=== Step 3: Trajectory Optimization ===\n');
optimizedPath = processor.optimizePCTTrajectory(rawPath, processor.robotParams);

% Visualize results
fprintf('\n=== Step 4: Visualization ===\n');

% Collect scene parameters for visualization
sceneParams = struct();
sceneParams.STEP_H = STEP_H;
sceneParams.STEP_D = STEP_D;
sceneParams.STEP_W = STEP_W;
sceneParams.N = N;

% Show trajectory in original scene context
processor.visualizeTrajectoryInScene(rawPath, optimizedPath, sceneParams);

% Show original vs optimized path comparison
processor.visualizeOptimizedTrajectory(rawPath, optimizedPath);

% Calculate path metrics
rawDist = sum(sqrt(sum(diff(rawPath).^2, 2)));
optDist = sum(sqrt(sum(diff(optimizedPath).^2, 2)));
heightGain = endPos(3) - startPos(3);

fprintf('\nPath Metrics:\n');
fprintf('Start position: [%.2f, %.2f, %.2f]\n', startPos(1), startPos(2), startPos(3));
fprintf('End position: [%.2f, %.2f, %.2f]\n', endPos(1), endPos(2), endPos(3));
fprintf('Height gain: %.2f m\n', heightGain);
fprintf('Original path length: %.2f m\n', rawDist);
fprintf('Optimized path length: %.2f m\n', optDist);
fprintf('Path length reduction: %.1f%%\n', (1 - optDist/rawDist) * 100);
fprintf('Average step height: %.2f m\n', heightGain / N);

% Save results
if ~exist('results', 'dir')
    mkdir('results');
end

save('results/stair_navigation_results.mat', ...
    'rawPath', 'optimizedPath', 'startPos', 'endPos', ...
    'STEP_H', 'STEP_W', 'STEP_D', 'N', 'NOISE');

fprintf('\nResults saved to results/stair_navigation_results.mat\n'); 