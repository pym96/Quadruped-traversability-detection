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
PTS_TOT = 5e4;   % total points to sample - increased for better density

% --- Helper lambda to sample cube points ---
makeCube = @(ctr,dim,n) [rand(n,1)*(dim(1))+ctr(1)-dim(1)/2, ...
                         rand(n,1)*(dim(2))+ctr(2)-dim(2)/2, ...
                         rand(n,1)*(dim(3))+ctr(3)-dim(3)/2];

% Pre-allocate fraction of points per element (rough)
numElems = 2*N + 2;              % N up + N down + 2 platforms
ptsPer   = max(1,round(PTS_TOT/numElems));
P = [];

%% 1. initial ground platform
plat0_dim = [STEP_W, STEP_D, 0.01];
plat0_ctr = [0, STEP_D/2, -plat0_dim(3)/2];
P         = [P; makeCube(plat0_ctr, plat0_dim, ptsPer)];

%% 2. ascending staircase
for i = 1:N
    topZ = i*STEP_H;
    dim  = [STEP_W, STEP_D, topZ];
    ctr  = [0, STEP_D*(i+0.5), topZ/2];
    P    = [P; makeCube(ctr, dim, ptsPer)];
end

%% 3. mid platform
platM_dim = [STEP_W, STEP_D, 0.01];
platM_ctr = [0, STEP_D*(N+1.5), N*STEP_H + platM_dim(3)/2];
P         = [P; makeCube(platM_ctr, platM_dim, ptsPer)];

%% 4. descending staircase
for j = 1:N
    topZ = (N-j)*STEP_H;
    dim  = [STEP_W, STEP_D, topZ + STEP_H];
    ctr  = [0, STEP_D*(N+1.5)+STEP_D*j - STEP_D/2, (topZ+STEP_H)/2];
    P    = [P; makeCube(ctr, dim, ptsPer)];
end

%% 5. final ground platform
platF_dim = [STEP_W, STEP_D, 0.01];
platF_ctr = [0, STEP_D*(2*N+3)/2, -platF_dim(3)/2];
P         = [P; makeCube(platF_ctr, platF_dim, ptsPer)];

%% Add noise and create pointCloud object
P = P + NOISE*randn(size(P));
pcd = pointCloud(P);

figure; pcshow(pcd); axis equal; xlabel X; ylabel Y; zlabel Z;
title('Minimal stair environment');

%% Save to MAT for planner scripts
save('simple_env.mat','pcd','STEP_W','STEP_H','STEP_D','N'); 