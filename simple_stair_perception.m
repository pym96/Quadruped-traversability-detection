%% Simple Stair Perception - Generate Synthetic Stair Point Cloud
% This script generates a synthetic stair environment for testing

clear;

% Stair scene parameters
N = 5;          % Number of steps (up and down)
STEP_W = 0.4;   % Step width (m)
STEP_H = 0.15;  % Step height (m)
STEP_D = 0.3;   % Step depth (m)
NOISE = 0.01;   % Sensor noise level (m)

fprintf('Generating synthetic stair scene...\n');
fprintf('Steps: %d up & down\n', N);
fprintf('Step dimensions: %.2fm x %.2fm x %.2fm (W x H x D)\n', STEP_W, STEP_H, STEP_D);

% Point cloud generation parameters
pointsPerMeter2 = 200;  % Point density per square meter

% Generate ascending stairs
points = [];

% Ascending stairs (going up)
for i = 1:N
    % Step surface
    stepZ = i * STEP_H;
    stepYmin = (i-1) * STEP_D;
    stepYmax = i * STEP_D;
    
    % Generate points on step surface
    numPointsStep = ceil(STEP_W * STEP_D * pointsPerMeter2);
    x_step = rand(numPointsStep, 1) * STEP_W;
    y_step = stepYmin + rand(numPointsStep, 1) * (stepYmax - stepYmin);
    z_step = ones(numPointsStep, 1) * stepZ;
    
    % Add to point cloud
    points = [points; x_step, y_step, z_step];
    
    % Step riser (vertical face)
    if i > 1
        numPointsRiser = ceil(STEP_W * STEP_H * pointsPerMeter2 * 2);
        x_riser = rand(numPointsRiser, 1) * STEP_W;
        y_riser = ones(numPointsRiser, 1) * stepYmin;
        z_riser = (i-1) * STEP_H + rand(numPointsRiser, 1) * STEP_H;
        
        points = [points; x_riser, y_riser, z_riser];
    end
end

% Platform at the top
platformYmin = N * STEP_D;
platformYmax = N * STEP_D + STEP_D;
platformZ = N * STEP_H;

numPointsPlatform = ceil(STEP_W * STEP_D * pointsPerMeter2);
x_platform = rand(numPointsPlatform, 1) * STEP_W;
y_platform = platformYmin + rand(numPointsPlatform, 1) * (platformYmax - platformYmin);
z_platform = ones(numPointsPlatform, 1) * platformZ;

points = [points; x_platform, y_platform, z_platform];

% Descending stairs (going down)
startY = N * STEP_D + STEP_D;
for i = 1:N
    % Step surface
    stepZ = (N-i+1) * STEP_H;
    stepYmin = startY + (i-1) * STEP_D;
    stepYmax = startY + i * STEP_D;
    
    % Generate points on step surface
    numPointsStep = ceil(STEP_W * STEP_D * pointsPerMeter2);
    x_step = rand(numPointsStep, 1) * STEP_W;
    y_step = stepYmin + rand(numPointsStep, 1) * (stepYmax - stepYmin);
    z_step = ones(numPointsStep, 1) * stepZ;
    
    % Add to point cloud
    points = [points; x_step, y_step, z_step];
    
    % Step riser (vertical face) - only if not the last step
    if i < N
        numPointsRiser = ceil(STEP_W * STEP_H * pointsPerMeter2 * 2);
        x_riser = rand(numPointsRiser, 1) * STEP_W;
        y_riser = ones(numPointsRiser, 1) * stepYmax;
        z_riser = (N-i) * STEP_H + rand(numPointsRiser, 1) * STEP_H;
        
        points = [points; x_riser, y_riser, z_riser];
    end
end

% Add sensor noise
noise = randn(size(points)) * NOISE;
points = points + noise;

% Create point cloud object
pcd = pointCloud(points);

fprintf('Generated point cloud with %d points\n', pcd.Count);
fprintf('Point cloud bounds:\n');
fprintf('  X: [%.2f, %.2f] m\n', min(points(:,1)), max(points(:,1)));
fprintf('  Y: [%.2f, %.2f] m\n', min(points(:,2)), max(points(:,2)));
fprintf('  Z: [%.2f, %.2f] m\n', min(points(:,3)), max(points(:,3)));

% Visualize the generated scene
figure('Name', 'Generated Stair Scene');
pcshow(pcd);
title('Synthetic Stair Point Cloud');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);
grid on; 