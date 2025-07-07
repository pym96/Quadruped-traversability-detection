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

% --- Helper functions to sample surface points ---
% Function to sample points on a rectangular surface (returns n x 3 matrix)
makeSurface = @(ctr,width,depth,n) [
    ctr(1)+(rand(n,1)-0.5).*width, ...
    ctr(2)+(rand(n,1)-0.5).*depth, ...
    repmat(ctr(3),[n,1])];

% Initialize point cloud matrix (n x 3)
P = zeros(0,3);

%% 1. initial ground platform
plat0_area = STEP_W * STEP_D;
plat0_pts = max(10, round(plat0_area * PTS_PER_M2));
P = [P; makeSurface([0, STEP_D/2, 0], STEP_W, STEP_D, plat0_pts)];

%% 2. ascending staircase
for i = 1:N
    % Horizontal surface (step top)
    step_top_area = STEP_W * STEP_D;
    step_top_pts = max(10, round(step_top_area * PTS_PER_M2));
    P = [P; makeSurface([0, STEP_D*(i+0.5), i*STEP_H], STEP_W, STEP_D, step_top_pts)];
    
    % Vertical surface (step riser)
    step_riser_area = STEP_W * STEP_H;
    step_riser_pts = max(10, round(step_riser_area * PTS_PER_M2));
    riser_y = STEP_D*(i+0.5) - STEP_D/2;
    riser_points = [
        (rand(step_riser_pts,1)-0.5).*STEP_W, ...
        repmat(riser_y,[step_riser_pts,1]), ...
        (i-1)*STEP_H + rand(step_riser_pts,1).*STEP_H];
    P = [P; riser_points];
end

%% 3. top platform at apex
topPlat_area = STEP_W * STEP_D;
topPlat_pts  = max(10, round(topPlat_area * PTS_PER_M2));
topPlat_y    = STEP_D * (N + 1.5);  % center Y between up and down stairs
P = [P; makeSurface([0, topPlat_y, N*STEP_H], STEP_W, STEP_D, topPlat_pts)];

%% 4. descending staircase
for j = 1:N
    % Horizontal surface (step top)
    step_top_area = STEP_W * STEP_D;
    step_top_pts = max(10, round(step_top_area * PTS_PER_M2));
    step_y = STEP_D*(N + 1.5 + j);      % Center of the descending step top
    step_z = (N - j) * STEP_H;          % Height of the descending step top
    P = [P; makeSurface([0, step_y, step_z], STEP_W, STEP_D, step_top_pts)];
    
    % Vertical surface (step riser)
    step_riser_area = STEP_W * STEP_H;
    step_riser_pts = max(10, round(step_riser_area * PTS_PER_M2));
    riser_y = step_y - STEP_D/2;        % Riser located at front face of the step
    riser_points = [
        (rand(step_riser_pts,1)-0.5).*STEP_W, ...
        repmat(riser_y,[step_riser_pts,1]), ...
        step_z + rand(step_riser_pts,1).*STEP_H];
    P = [P; riser_points];
end

%% 5. final ground platform (after descending staircase)
platF_area = STEP_W * STEP_D;
platF_pts = max(10, round(platF_area * PTS_PER_M2));
P = [P; makeSurface([0, STEP_D*(2*N + 2.5), 0], STEP_W, STEP_D, platF_pts)];

%% 6. Add ceiling
total_length = STEP_D * (2*N + 3);  % overall length unchanged (gap maintained)
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