%% Analyze stair traversability
% This script combines the simple stair scene with our traversability analysis

clear; clc;

% First generate the stair scene
run('simple_stair_perception.m');

% Initialize the tomogram processor with fine resolution
ds = 0.10;  % 10cm slice interval
rg = 0.05; % 5cm grid resolution
processor = TomogramProcessor(ds, rg);

% Load the point cloud
processor.loadPointCloud(pcd.Location);

% Process tomograms
processor.processTomograms();

% Visualize the results
processor.visualizeSlices();

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