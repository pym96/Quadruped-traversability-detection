%% simple_stair_perception.m
% Minimal point-cloud environment: N ascending steps -> mid platform -> N descending steps.
% Generates point cloud `pcd` and saves to simple_env.mat.

clear; clc;

% --- Parameters (edit freely) ---
STEP_W  = 1.0;   % step width   (m)
STEP_H  = 0.20;  % step height  (m)
STEP_D  = 0.30;  % step depth   (m)
N       = 6;     % number of steps up (and down)
NOISE   = 0.005; % std-dev Gaussian noise (m) - reduced for better analysis
PTS_PER_M2 = 1000; % points per square meter for surface sampling
CEILING_H = 2.0; % ceiling height above ground (m)

% Initialize point cloud as empty matrix with 3 columns (x,y,z)
P = zeros(0, 3);

%% 1. initial ground platform (removed)
plat0_area = STEP_W * STEP_D;
plat0_pts = max(10, round(plat0_area * PTS_PER_M2));
P = [P; makeSurface([0, STEP_D/2, 0], STEP_W, STEP_D, plat0_pts)];

%% 2. ascending staircase (left side - going up)
for i = 1:N
    % Horizontal surface (step top)
    step_top_area = STEP_W * STEP_D;
    step_top_pts = max(10, round(step_top_area * PTS_PER_M2));
    P = [P; makeSurface([0, STEP_D*(i+0.5), i*STEP_H], STEP_W, STEP_D, step_top_pts)];
    
    % Vertical surface (step riser)
    step_riser_area = STEP_W * STEP_H;
    step_riser_pts = max(10, round(step_riser_area * PTS_PER_M2));
    riser_y = STEP_D*(i+0.5) - STEP_D/2;
    riser_x = (rand(step_riser_pts,1)-0.5)*STEP_W;    % X coordinates
    riser_y_coords = repmat(riser_y, [step_riser_pts,1]);    % Y coordinates (fixed)
    riser_z = (i-1)*STEP_H + rand(step_riser_pts,1)*STEP_H;  % Z coordinates
    riser_points = [riser_x, riser_y_coords, riser_z];  % n x 3 matrix
    P = [P; riser_points];
end

%% 3. high mid platform
platM_area = STEP_W * STEP_D;
platM_pts = max(10, round(platM_area * PTS_PER_M2));
P = [P; makeSurface([0, STEP_D*(N+1.5), N*STEP_H], STEP_W, STEP_D, platM_pts)];

%% 4. descending staircase (right side - going down)
for j = 1:N
    % Horizontal surface (step top)
    step_top_area = STEP_W * STEP_D;
    step_top_pts = max(10, round(step_top_area * PTS_PER_M2));
    % Adjust position so first descending step's left edge aligns with platform's right edge
    step_y = STEP_D*(N+1.5) + STEP_D/2 + STEP_D*j - STEP_D/2;  % Add STEP_D/2 offset
    step_z = (N-j)*STEP_H;
    P = [P; makeSurface([0, step_y, step_z], STEP_W, STEP_D, step_top_pts)];
    
    % Vertical surface (step riser) - connecting platform/previous step to current step
    step_riser_area = STEP_W * STEP_H;
    step_riser_pts = max(10, round(step_riser_area * PTS_PER_M2));
    riser_y = step_y - STEP_D/2;  % Back of the step (connecting to previous level)
    riser_x = (rand(step_riser_pts,1)-0.5)*STEP_W;    % X coordinates
    riser_y_coords = repmat(riser_y, [step_riser_pts,1]);    % Y coordinates (fixed)
    
    if j == 1
        % First descending step: connect from high platform to first step
        riser_z = step_z + rand(step_riser_pts,1)*STEP_H;  % From step height to platform height
    else
        % Other steps: connect from previous step to current step
        prev_step_z = (N-(j-1))*STEP_H;
        riser_z = step_z + rand(step_riser_pts,1)*(prev_step_z - step_z);  % From current to previous step height
    end
    
    riser_points = [riser_x, riser_y_coords, riser_z];  % n x 3 matrix
    P = [P; riser_points];
end

%% 5. final ground platform
% platF_area = STEP_W * STEP_D;
% platF_pts = max(10, round(platF_area * PTS_PER_M2));
% P = [P; makeSurface([0, STEP_D*(2*N+3)/2, 0], STEP_W, STEP_D, platF_pts)];

%% 6. Add ceiling
total_length = STEP_D * (2*N + 3);  % N up + N down + 3 platforms
ceiling_area = STEP_W * total_length;
ceiling_pts = max(10, round(ceiling_area * PTS_PER_M2));
P = [P; makeSurface([0, total_length/2, CEILING_H], STEP_W, total_length, ceiling_pts)];

%% Add noise and create pointCloud object
P = P + NOISE*randn(size(P));
pcd = pointCloud(P);

% Visualize
figure('Name', 'Stair Point Cloud');
pcshow(pcd); 
axis equal; 
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Stair environment with surface points only');
view(45, 30);  % Set view angle for better visualization

%% Save to MAT for planner scripts
save('simple_env.mat','pcd','STEP_W','STEP_H','STEP_D','N','CEILING_H');

% --- Helper functions to sample surface points ---
% Function to sample points on a rectangular surface
function points = makeSurface(ctr, width, depth, n)
    % Create n random points on the surface
    x_coords = ctr(1) + (rand(n,1)-0.5)*width;     % X coordinates (n x 1)
    y_coords = ctr(2) + (rand(n,1)-0.5)*depth;     % Y coordinates (n x 1)  
    z_coords = repmat(ctr(3), [n,1]);              % Z coordinates (n x 1)
    
    % Concatenate horizontally to create n x 3 matrix
    points = [x_coords, y_coords, z_coords];
end 