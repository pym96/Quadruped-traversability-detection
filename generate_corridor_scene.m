% Generate a corridor scene with walls, ceiling and stairs at the end
% The scene will be represented as a point cloud

%% Scene Parameters
% Corridor dimensions
corridor_length = 8.0;    % meters
corridor_width = 2.0;     % meters
corridor_height = 2.5;    % meters
resolution = 0.05;        % 5cm grid

% Stairs parameters
num_stairs = 5;
stair_height = 0.15;      % meters
stair_depth = 0.30;       % meters
stairs_width = 1.6;       % slightly narrower than corridor

%% Generate Base Points
% Initialize point cloud
points = [];

% Generate floor points (including stairs)
[X, Y] = meshgrid(-1:resolution:corridor_length, -corridor_width/2:resolution:corridor_width/2);
Z = zeros(size(X));

% Add stairs
for i = 1:num_stairs
    stair_start = (i-1) * stair_depth;
    mask = X >= stair_start & X < (stair_start + stair_depth);
    Z(mask) = (i-1) * stair_height;
end

% Add floor points with small random jitter for more natural edges
jitter = 0.002;  % 2mm random variation
floor_points = [X(:), Y(:), Z(:) + jitter * randn(numel(X), 1)];
points = [points; floor_points];

% Generate side walls (only walls, no end caps)
wall_resolution = 0.1;  % 10cm grid for walls
[X_wall, Z_wall] = meshgrid(-1:wall_resolution:corridor_length, 0:wall_resolution:corridor_height);

% Left wall
Y_left = ones(size(X_wall)) * (-corridor_width/2);
left_wall = [X_wall(:), Y_left(:), Z_wall(:)];

% Right wall
Y_right = ones(size(X_wall)) * (corridor_width/2);
right_wall = [X_wall(:), Y_right(:), Z_wall(:)];

% Add walls with small random jitter
wall_jitter = 0.002;
points = [points; 
         left_wall + wall_jitter * randn(size(left_wall));
         right_wall + wall_jitter * randn(size(right_wall))];
     
% Generate ceiling
[X_ceiling, Y_ceiling] = meshgrid(-1:resolution:corridor_length, -corridor_width/2:resolution:corridor_width/2);
Z_ceiling = ones(size(X_ceiling)) * corridor_height;

% Add ceiling points with jitter
ceiling_points = [X_ceiling(:), Y_ceiling(:), Z_ceiling(:) + jitter * randn(numel(X_ceiling), 1)];
points = [points; ceiling_points];

%% Process Point Cloud
% Create point cloud visualization
figure('Name', 'Corridor Scene with Stairs');

% 3D scatter plot of the point cloud
subplot(1,2,1)
scatter3(points(:,1), points(:,2), points(:,3), '.', 'MarkerEdgeAlpha', 0.1);
title('Original Point Cloud');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal;
view(45, 30);
grid on;

% Top view
subplot(1,2,2)
scatter(points(:,1), points(:,2), '.', 'MarkerEdgeAlpha', 0.1);
title('Top View');
xlabel('X (m)'); ylabel('Y (m)');
axis equal;
grid on;

%% Process Tomograms
% Create and process tomograms
tomogram = TomogramProcessor(0.20, 0.10);  % 10cm slice interval, 5cm grid resolution
tomogram = tomogram.loadPointCloud(points);
tomogram = tomogram.processTomograms();

% Visualize all tomogram slices
tomogram.visualizeSlices();

% Save the processed data
save('corridor_scene.mat', 'points', 'tomogram');

% Print scene statistics
fprintf('\nScene Statistics:\n');
fprintf('Corridor length: %.1f m\n', corridor_length);
fprintf('Corridor width: %.1f m\n', corridor_width);
fprintf('Corridor height: %.1f m\n', corridor_height);
fprintf('Number of stairs: %d\n', num_stairs);
fprintf('Stair height: %.2f m\n', stair_height);
fprintf('Stair depth: %.2f m\n', stair_depth);
fprintf('Number of points: %d\n', size(points, 1));
fprintf('Number of tomogram slices: %d\n', tomogram.N + 1); 