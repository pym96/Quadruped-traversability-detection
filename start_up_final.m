% Startup script for quadruped robot navigation
% This script initializes the workspace and adds necessary paths

% Clear workspace and command window
clear all;
close all;
clc;

% Add current directory to path
current_dir = pwd;
addpath(current_dir);

% Check if required files exist
required_files = {'terrain_point_cloud_processor.m', 'generate_corridor_scene.m'};
missing_files = {};

for i = 1:length(required_files)
    if ~exist(required_files{i}, 'file')
        missing_files{end+1} = required_files{i};
    end
end

if ~isempty(missing_files)
    fprintf('Error: The following required files are missing:\n');
    for i = 1:length(missing_files)
        fprintf('  - %s\n', missing_files{i});
    end
    error('Please ensure all required files are in the current directory.');
end

% Initialize workspace
fprintf('Initializing quadruped navigation workspace...\n');

% Run the scene generation
fprintf('Generating corridor scene...\n');
generate_corridor_scene;

fprintf('\nWorkspace initialized successfully!\n');
fprintf('You can now use the following main functions:\n');
fprintf('  - generate_corridor_scene: Creates a new corridor scene\n');
fprintf('  - TerrainPointCloudProcessor: Processes point cloud data\n');
