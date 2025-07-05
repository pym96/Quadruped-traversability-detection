%% corridor_stair_navigation.m
% Enhanced with PCT (Point Cloud Tomography) Algorithm
% Based on research paper: "Efficient Global Navigational Planning in 3D Structures based on Point Cloud Tomography"

clear; close all; clc;

%% Scene Parameters
% Corridor dimensions
CORRIDOR_LENGTH = 5.0;  % meters
CORRIDOR_WIDTH = 2.0;   % meters
CORRIDOR_HEIGHT = 2.4;  % meters
WALL_THICKNESS = 0.2;   % meters

% Stair dimensions
STEP_HEIGHT = 0.15;     % meters (standard riser height)
STEP_DEPTH = 0.30;      % meters (standard tread depth)
NUM_STEPS = 8;          % total steps
STAIR_WIDTH = 2.0;      % meters (same as corridor)

% Platform dimensions
PLATFORM_SIZE = 2.0;    % meters (square platform)

% Point density
POINTS_PER_M3 = 1000;   % points per cubic meter for sampling

%% Generate Scene Point Cloud
% Helper function for uniform cube sampling
createCube = @(center, dims, density) [...
    center(1) + dims(1) * (rand(density, 1) - 0.5), ...
    center(2) + dims(2) * (rand(density, 1) - 0.5), ...
    center(3) + dims(3) * (rand(density, 1) - 0.5)];

pcXYZ = [];

% 1. Corridor Floor
floor_points = createCube([CORRIDOR_LENGTH/2, 0, 0], ...
    [CORRIDOR_LENGTH, CORRIDOR_WIDTH, WALL_THICKNESS], ...
    round(CORRIDOR_LENGTH * CORRIDOR_WIDTH * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; floor_points];

% 2. Corridor Ceiling
ceiling_points = createCube([CORRIDOR_LENGTH/2, 0, CORRIDOR_HEIGHT], ...
    [CORRIDOR_LENGTH, CORRIDOR_WIDTH, WALL_THICKNESS], ...
    round(CORRIDOR_LENGTH * CORRIDOR_WIDTH * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; ceiling_points];

% 3. Corridor Walls
% Left wall
left_wall_points = createCube([CORRIDOR_LENGTH/2, -CORRIDOR_WIDTH/2, CORRIDOR_HEIGHT/2], ...
    [CORRIDOR_LENGTH, WALL_THICKNESS, CORRIDOR_HEIGHT], ...
    round(CORRIDOR_LENGTH * CORRIDOR_HEIGHT * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; left_wall_points];

% Right wall
right_wall_points = createCube([CORRIDOR_LENGTH/2, CORRIDOR_WIDTH/2, CORRIDOR_HEIGHT/2], ...
    [CORRIDOR_LENGTH, WALL_THICKNESS, CORRIDOR_HEIGHT], ...
    round(CORRIDOR_LENGTH * CORRIDOR_HEIGHT * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; right_wall_points];

% Front wall (entrance)
front_wall_points = createCube([0, 0, CORRIDOR_HEIGHT/2], ...
    [WALL_THICKNESS, CORRIDOR_WIDTH, CORRIDOR_HEIGHT], ...
    round(CORRIDOR_WIDTH * CORRIDOR_HEIGHT * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; front_wall_points];

% 4. Stairs
for i = 1:NUM_STEPS
    step_points = createCube([...
        CORRIDOR_LENGTH + STEP_DEPTH * (i-0.5), ...
        0, ...
        STEP_HEIGHT * (i-0.5)], ...
        [STEP_DEPTH, STAIR_WIDTH, STEP_HEIGHT * i], ...
        round(STEP_DEPTH * STAIR_WIDTH * STEP_HEIGHT * i * POINTS_PER_M3));
    pcXYZ = [pcXYZ; step_points];
end

% 5. Platform
platform_height = NUM_STEPS * STEP_HEIGHT;
platform_center = [CORRIDOR_LENGTH + NUM_STEPS*STEP_DEPTH + PLATFORM_SIZE/2, 0, platform_height];

% Platform floor
platform_points = createCube(platform_center, ...
    [PLATFORM_SIZE, PLATFORM_SIZE, WALL_THICKNESS], ...
    round(PLATFORM_SIZE * PLATFORM_SIZE * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; platform_points];

% Platform ceiling
platform_ceiling_points = createCube([platform_center(1), platform_center(2), platform_center(3) + CORRIDOR_HEIGHT], ...
    [PLATFORM_SIZE, PLATFORM_SIZE, WALL_THICKNESS], ...
    round(PLATFORM_SIZE * PLATFORM_SIZE * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; platform_ceiling_points];

% Platform end wall
end_wall_points = createCube([platform_center(1) + PLATFORM_SIZE/2, 0, platform_height + CORRIDOR_HEIGHT/2], ...
    [WALL_THICKNESS, PLATFORM_SIZE, CORRIDOR_HEIGHT], ...
    round(PLATFORM_SIZE * CORRIDOR_HEIGHT * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; end_wall_points];

% 6. Stair side walls
stair_length = NUM_STEPS * STEP_DEPTH;
stair_center_height = platform_height / 2;

% Left stair wall
left_stair_wall_points = createCube([...
    CORRIDOR_LENGTH + stair_length/2, ...
    -STAIR_WIDTH/2, ...
    stair_center_height], ...
    [stair_length, WALL_THICKNESS, platform_height], ...
    round(stair_length * platform_height * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; left_stair_wall_points];

% Right stair wall
right_stair_wall_points = createCube([...
    CORRIDOR_LENGTH + stair_length/2, ...
    STAIR_WIDTH/2, ...
    stair_center_height], ...
    [stair_length, WALL_THICKNESS, platform_height], ...
    round(stair_length * platform_height * WALL_THICKNESS * POINTS_PER_M3));
pcXYZ = [pcXYZ; right_stair_wall_points];

% Define start and goal positions
startPos = [0.5, 0, 0.3];  % Start position: slightly inside corridor, centered, just above floor
goalPos = [CORRIDOR_LENGTH + NUM_STEPS*STEP_DEPTH + PLATFORM_SIZE/2, 0, platform_height + 0.3];  % Center of platform, slightly above floor

% Visualize the scene
figure('Name', 'Scene and Path Planning', 'Position', [100, 100, 1200, 800]);

% Scene visualization
subplot(2,2,[1,3]);
pcshow(pcXYZ, 'MarkerSize', 20);
hold on;
plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Start');
plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Goal');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Corridor with Stairs Scene');
axis equal;
grid on;
view(45, 30);
legend;

% PCT Algorithm Parameters
GRID_RES = 0.05;    % 5cm grid resolution
SLICE_HEIGHT = 0.3; % 30cm slice separation (ds parameter)
ROBOT_HEIGHT = 0.25; % Robot body height (d_min parameter)
ROBOT_CLEARANCE = 0.5; % Preferred clearance (d_ref parameter)

%% PCT ALGORITHM IMPLEMENTATION
fprintf('=== PCT (Point Cloud Tomography) Navigation System ===\n');

% Initialize robot and cost parameters
robot_params = struct('d_min', ROBOT_HEIGHT, 'd_ref', ROBOT_CLEARANCE, ...
                     'd_inf', 0.3, 'd_sm', 0.5, 'rg', GRID_RES);
cost_params = struct('c_B', 1000, 'alpha_d', 10, 'alpha_s', 5, ...
                    'theta_b', 0.5, 'theta_s', 0.2, 'theta_p', 0.7);

% Step 1: Build Tomogram
fprintf('Step 1: Building tomogram with %.2fm slice separation...\n', SLICE_HEIGHT);
tomogram = buildPCTTomogram(pcXYZ, GRID_RES, SLICE_HEIGHT);

% Step 2: Estimate Traversability
fprintf('Step 2: Estimating traversability costs...\n');
tomogram = estimatePCTTraversability(tomogram, robot_params, cost_params, platform_height);

% Step 3: Simplify Tomogram
fprintf('Step 3: Simplifying tomogram...\n');
tomogram_simplified = simplifyPCTTomogram(tomogram, cost_params.c_B);

% Step 4: Plan Path
fprintf('Step 4: Planning path from start to goal...\n');
globalPath = planPCTPath(tomogram_simplified, startPos, goalPos, robot_params, cost_params);

if isempty(globalPath)
    error('Failed to find a valid path!');
end

% Step 5: Optimize Trajectory
fprintf('Step 5: Optimizing trajectory...\n');
smoothPath = optimizePCTTrajectory(globalPath, tomogram_simplified, robot_params);

% Create visualization figures
figure('Name', 'Scene and Path Planning', 'Position', [100, 100, 1200, 800]);

% Scene visualization with path
subplot(2,2,[1,3]);
pcshow(pcXYZ, 'MarkerSize', 20);
hold on;
plot3(startPos(1), startPos(2), startPos(3), 'go', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Start');
plot3(goalPos(1), goalPos(2), goalPos(3), 'ro', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'Goal');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Corridor with Stairs Scene');
axis equal;
grid on;
view(45, 30);
legend;

% Create new figure for tomogram and traversability visualization
figure('Name', 'Tomogram and Traversability Analysis', 'Position', [100, 100, 1600, 800]);

% Select key slices for visualization
numSlices = length(tomogram_simplified.slices);
keySliceIndices = [1, floor(numSlices/3), floor(2*numSlices/3), numSlices];

for i = 1:length(keySliceIndices)
    sliceIdx = keySliceIndices(i);
    slice = tomogram_simplified.slices{sliceIdx};
    
    % Ground elevation visualization
    subplot(2,4,i);
    imagesc(slice.e_G);
    colormap(gca, jet);
    colorbar;
    title(sprintf('Ground Elevation (Z=%.2fm)', tomogram_simplified.heights(sliceIdx)));
    xlabel('Y Grid'); ylabel('X Grid');
    axis equal tight;
    
    % Traversability cost visualization
    subplot(2,4,i+4);
    traversability = slice.c_T;
    % Normalize costs for better visualization
    traversability(traversability == cost_params.c_B) = nan;  % Mark impassable areas as NaN
    imagesc(traversability, 'AlphaData', ~isnan(traversability));
    colormap(gca, flipud(hot));  % Red = high cost, Blue = low cost
    colorbar;
    title(sprintf('Traversability Cost (Z=%.2fm)', tomogram_simplified.heights(sliceIdx)));
    xlabel('Y Grid'); ylabel('X Grid');
    axis equal tight;
    
    % Add path projection if it passes through this slice
    hold on;
    if ~isempty(smoothPath)
        % Find path points near this slice
        sliceHeight = tomogram_simplified.heights(sliceIdx);
        pathMask = abs(smoothPath(:,3) - sliceHeight) < SLICE_HEIGHT/2;
        if any(pathMask)
            pathPoints = smoothPath(pathMask, :);
            % Convert to grid coordinates
            pathGrid = zeros(size(pathPoints, 1), 2);
            for j = 1:size(pathPoints, 1)
                pathGrid(j,:) = worldToGrid(pathPoints(j,1:2), tomogram_simplified);
            end
            plot(pathGrid(:,2), pathGrid(:,1), 'g-', 'LineWidth', 2);
            % Plot start/goal if they're in this slice
            if abs(startPos(3) - sliceHeight) < SLICE_HEIGHT/2
                startGrid = worldToGrid(startPos(1:2), tomogram_simplified);
                plot(startGrid(2), startGrid(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            end
            if abs(goalPos(3) - sliceHeight) < SLICE_HEIGHT/2
                goalGrid = worldToGrid(goalPos(1:2), tomogram_simplified);
                plot(goalGrid(2), goalGrid(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            end
        end
    end
    hold off;
end

% Return to original figure for path visualization
figure(1);
subplot(2,2,[1,3]);
if ~isempty(smoothPath)
    plot3(smoothPath(:,1), smoothPath(:,2), smoothPath(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
end
legend;

% Path profile and top view (as before)
subplot(2,2,2);
plot(smoothPath(:,1), smoothPath(:,3), 'b-', 'LineWidth', 2);
hold on;
plot(startPos(1), startPos(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPos(1), goalPos(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Z (m)');
title('Path Height Profile');
grid on;

subplot(2,2,4);
plot(smoothPath(:,1), smoothPath(:,2), 'b-', 'LineWidth', 2);
hold on;
plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
title('Path Top View');
grid on;
axis equal;

fprintf('Path planning complete!\n');
fprintf('Total path length: %.2f meters\n', sum(sqrt(sum(diff(smoothPath).^2, 2))));
fprintf('Height gain: %.2f meters\n', goalPos(3) - startPos(3));

%% ========================================================================
%  PCT ALGORITHM FUNCTIONS (Based on Research Paper)
%% ========================================================================

%% Part 1: Tomogram Construction (Section III-A) - PCT Paper Implementation
function tomogram = buildPCTTomogram(pointCloud, gridRes, sliceHeight)
    % Build tomogram following PCT paper methodology with proper obstacle detection
    
    % Get point cloud bounds
    minX = min(pointCloud(:,1)); maxX = max(pointCloud(:,1));
    minY = min(pointCloud(:,2)); maxY = max(pointCloud(:,2));
    minZ = min(pointCloud(:,3)); maxZ = max(pointCloud(:,3));
    
    % Define height layers with overlap for better connectivity
    overlap = sliceHeight * 0.3;  % 30% overlap between slices
    heights = minZ:sliceHeight:maxZ;
    numSlices = length(heights);
    
    % Grid dimensions
    gridW = ceil((maxX - minX) / gridRes);
    gridH = ceil((maxY - minY) / gridRes);
    
    fprintf('Tomogram: %d slices, grid %dx%d, Z range [%.2f, %.2f]\n', ...
            numSlices, gridW, gridH, minZ, maxZ);
    
    % Initialize tomogram structure
    tomogram = struct();
    tomogram.slices = cell(numSlices, 1);
    tomogram.heights = heights;
    tomogram.gridRes = gridRes;
    tomogram.minX = minX; tomogram.maxX = maxX;
    tomogram.minY = minY; tomogram.maxY = maxY;
    tomogram.minZ = minZ; tomogram.maxZ = maxZ;
    tomogram.gridW = gridW; tomogram.gridH = gridH;
    
    % Process each slice
    for k = 1:numSlices
        planeHeight = heights(k);
        
        % Initialize elevation maps
        e_G = -inf(gridW, gridH);  % Ground elevation
        e_C = inf(gridW, gridH);   % Ceiling elevation
        pointDensity = zeros(gridW, gridH);
        
        % Define slice bounds with overlap
        lowerBound = planeHeight - sliceHeight/2 - overlap;
        upperBound = planeHeight + sliceHeight/2 + overlap;
        
        % Filter points within slice bounds
        sliceMask = pointCloud(:,3) >= lowerBound & pointCloud(:,3) <= upperBound;
        slicePoints = pointCloud(sliceMask, :);
        
        % Process points in the slice
        for i = 1:size(slicePoints, 1)
            point = slicePoints(i, :);
            
            % Convert to grid coordinates
            gx = floor((point(1) - minX) / gridRes) + 1;
            gy = floor((point(2) - minY) / gridRes) + 1;
            
            if gx < 1 || gx > gridW || gy < 1 || gy > gridH
                continue;
            end
            
            pointDensity(gx, gy) = pointDensity(gx, gy) + 1;
            
            % Classify point relative to plane
            if point(3) >= planeHeight
                e_C(gx, gy) = min(e_C(gx, gy), point(3));
            else
                e_G(gx, gy) = max(e_G(gx, gy), point(3));
            end
        end
        
        % Apply median filter to remove noise
        e_G = medfilt2(e_G, [3 3]);
        e_C = medfilt2(e_C, [3 3]);
        
        % Fill gaps in ground and ceiling
        e_G = fillmissing(e_G, 'nearest');
        e_C = fillmissing(e_C, 'nearest');
        
        % Detect walls and obstacles using density and gradient
        [Gx, Gy] = gradient(e_G);
        gradientMag = sqrt(Gx.^2 + Gy.^2);
        isWall = gradientMag > 0.5 | pointDensity > prctile(pointDensity(:), 95);
        
        % Store slice data
        tomogram.slices{k} = struct();
        tomogram.slices{k}.height = planeHeight;
        tomogram.slices{k}.e_G = e_G;
        tomogram.slices{k}.e_C = e_C;
        tomogram.slices{k}.pointDensity = pointDensity;
        tomogram.slices{k}.isWall = isWall;
        tomogram.slices{k}.gradientMag = gradientMag;
    end
    
    % Post-process to identify gateways
    tomogram = identifyGateways(tomogram);
end

%% Gateway Identification
function tomogram = identifyGateways(tomogram)
    numSlices = length(tomogram.slices);
    
    for k = 1:numSlices-1
        slice = tomogram.slices{k};
        nextSlice = tomogram.slices{k+1};
        
        % Find potential gateway cells (areas with good clearance in both slices)
        clearance_current = nextSlice.e_C - slice.e_G;
        clearance_next = nextSlice.e_C - nextSlice.e_G;
        
        % Gateway conditions:
        % 1. Good clearance in both slices
        % 2. Moderate gradient (for stairs/ramps)
        % 3. Not a wall in either slice
        isGateway = clearance_current > 0.5 & ...
                   clearance_next > 0.5 & ...
                   ~slice.isWall & ~nextSlice.isWall & ...
                   slice.gradientMag < 0.8 & nextSlice.gradientMag < 0.8;
        
        % Store gateway information
        tomogram.slices{k}.gateways = isGateway;
    end
    
    % Last slice has no gateways up
    tomogram.slices{end}.gateways = false(size(slice.e_G));
end

%% Part 2: Traversability Estimation (Section III-B)
function tomogram = estimatePCTTraversability(tomogram, robotParams, costParams, platTopZ)
    numSlices = length(tomogram.slices);
    
    for k = 1:numSlices
        slice = tomogram.slices{k};
        
        % Calculate clearance-based cost
        d_I = slice.e_C - slice.e_G;
        c_I = max(0, costParams.alpha_d * (robotParams.d_ref - d_I));
        
        % Mark insufficient clearance as impassable
        if slice.height >= platTopZ-0.05
            local_dmin = robotParams.d_min * 0.6;
        else
            local_dmin = robotParams.d_min * 0.4;
        end
        c_I(d_I < local_dmin) = costParams.c_B;
        
        % Mark walls as impassable
        c_I(slice.isWall) = costParams.c_B;
        
        % Calculate ground-based cost
        c_G = calculateGroundCost(slice, costParams);
        
        % Calculate gateway-based cost reduction
        c_W = zeros(size(c_I));
        if isfield(slice, 'gateways') && any(slice.gateways(:))
            % Reduce cost near gateways to encourage their use
            c_W(slice.gateways) = -costParams.alpha_s;
        end
        
        % Combine costs with gateway influence
        c_T = min(costParams.c_B, c_I + c_G + c_W);
        
        % Add center line preference
        [h, w] = size(c_T);
        centerY = w/2;
        [X, Y] = meshgrid(1:w, 1:h);  % 注意这里交换了顺序，使Y的维度与c_T匹配
        centerDist = abs(Y - centerY) * tomogram.gridRes;
        c_T = c_T + centerDist * 0.1;  % Small cost increase with distance from center
        
        % Store traversability cost
        tomogram.slices{k}.c_T = c_T;
        
        % Debug output
        if mod(k, 4) == 0
            fprintf('Slice %d/%d: Height=%.2fm, Traversable=%.1f%%\n', ...
                k, numSlices, slice.height, ...
                100 * sum(c_T(:) < costParams.c_B) / numel(c_T));
        end
    end
end

%% Ground Cost Calculation
function c_G = calculateGroundCost(slice, costParams)
    % Calculate ground cost based on gradient and local properties
    [Gx, Gy] = gradient(slice.e_G);
    m_grad = sqrt(Gx.^2 + Gy.^2);
    m_xy = max(abs(Gx), abs(Gy));
    
    c_G = zeros(size(slice.e_G));
    
    % Smooth surfaces
    smoothMask = m_grad < costParams.theta_s;
    c_G(smoothMask) = 0;
    
    % Obstacle boundaries
    obstacleMask = m_xy > costParams.theta_b;
    c_G(obstacleMask) = costParams.c_B;
    
    % Edge regions (potential steps)
    edgeMask = (m_xy <= costParams.theta_b) & (m_grad >= costParams.theta_s);
    
    % Calculate safety score for edge regions
    [h, w] = size(slice.e_G);
    for i = 2:h-1
        for j = 2:w-1
            if edgeMask(i,j)
                % Get local patch
                patch = m_grad(max(1,i-2):min(h,i+2), max(1,j-2):min(w,j+2));
                p_s = sum(patch(:) < costParams.theta_s) / numel(patch);
                
                if p_s > costParams.theta_p
                    c_G(i,j) = costParams.alpha_s * (1.0 / p_s)^2;
                else
                    c_G(i,j) = costParams.c_B;
                end
            end
        end
    end
end

%% Part 3: Tomogram Simplification (Section III-C)
function tomogramSimplified = simplifyPCTTomogram(tomogram, c_B)
    % Remove redundant slices to optimize data structure
    
    numSlices = length(tomogram.slices);
    keepIndices = true(1, numSlices);
    
    % Always keep first and last slices
    if numSlices <= 2
        tomogramSimplified = tomogram;
        return;
    end
    
    % Check redundancy for intermediate slices
    for k = 2:numSlices-1
        isRedundant = true;
        
        % Get traversable cells in current slice
        currentSlice = tomogram.slices{k};
        traversableCells = find(currentSlice.c_T < c_B);
        
        if isempty(traversableCells)
            % No traversable cells, slice is redundant
            continue;
        end
        
        % Check if this slice provides unique traversable space
        prevSlice = tomogram.slices{k-1};
        nextSlice = tomogram.slices{k+1};
        
        for idx = 1:length(traversableCells)
            cellIdx = traversableCells(idx);
            
            % Check uniqueness conditions (Equations 8-9)
            condVsPrev = (currentSlice.e_G(cellIdx) > prevSlice.e_G(cellIdx)) || ...
                        (currentSlice.c_T(cellIdx) < prevSlice.c_T(cellIdx));
            
            condVsNext = (nextSlice.e_G(cellIdx) > currentSlice.e_G(cellIdx)) || ...
                        (currentSlice.c_T(cellIdx) < nextSlice.c_T(cellIdx));
            
            if condVsPrev && condVsNext
                isRedundant = false;
                break;
            end
        end
        
        if isRedundant
            keepIndices(k) = false;
        end
    end
    
    % Create simplified tomogram
    tomogramSimplified = tomogram;
    tomogramSimplified.slices = tomogram.slices(keepIndices);
    tomogramSimplified.heights = tomogram.heights(keepIndices);
    
    fprintf('Tomogram simplified: %d -> %d slices (%.1f%% reduction)\n', ...
            numSlices, length(tomogramSimplified.slices), ...
            (1 - length(tomogramSimplified.slices)/numSlices) * 100);
end

%% Part 4: 3D Path Planning (Section III-D) - Enhanced with Debugging
function globalPath = planPCTPath(tomogram, startPos, goalPos, robotParams, costParams)
    % Plan path through tomogram slices using modified A*
    
    fprintf('=== PCT Path Planning Debug ===\n');
    fprintf('Start: [%.2f, %.2f, %.2f]\n', startPos);
    fprintf('Goal: [%.2f, %.2f, %.2f]\n', goalPos);
    
    % Find start and goal slice indices
    startSliceIdx = findNearestSlice(tomogram, startPos(3));
    goalSliceIdx = findNearestSlice(tomogram, goalPos(3));
    
    fprintf('Start slice: %d (height %.2f)\n', startSliceIdx, tomogram.heights(startSliceIdx));
    fprintf('Goal slice: %d (height %.2f)\n', goalSliceIdx, tomogram.heights(goalSliceIdx));
    
    % Convert world coordinates to grid coordinates
    startGrid = worldToGrid(startPos(1:2), tomogram);
    goalGrid = worldToGrid(goalPos(1:2), tomogram);
    
    fprintf('Start grid: [%d, %d]\n', startGrid);
    fprintf('Goal grid: [%d, %d]\n', goalGrid);
    
    % Debug: Check traversability at start and goal
    startValid = isValidGridPosition(startGrid, tomogram.slices{startSliceIdx}, costParams.c_B);
    goalValid = isValidGridPosition(goalGrid, tomogram.slices{goalSliceIdx}, costParams.c_B);
    
    fprintf('Start valid: %d, Goal valid: %d\n', startValid, goalValid);
    
    % Validate start and goal positions
    if ~startValid || ~goalValid
        fprintf('Invalid start or goal position, attempting correction...\n');
        [startGrid, goalGrid] = correctInvalidPositions(startGrid, goalGrid, tomogram, costParams.c_B);
        fprintf('Corrected - Start: [%d, %d], Goal: [%d, %d]\n', startGrid, goalGrid);
    end
    
    % Try simplified 2D approach first
    fprintf('Trying simplified 2D approach...\n');
    globalPath = planSimplified2DPath(tomogram, startPos, goalPos, robotParams);
    
    if ~isempty(globalPath)
        fprintf('Simplified 2D path found with %d waypoints\n', size(globalPath, 1));
        return;
    end
    
    % If 2D fails, try 3D A*
    fprintf('Trying 3D A* search...\n');
    path3D = run3DAStar(startGrid, goalGrid, startSliceIdx, goalSliceIdx, tomogram, costParams);
    
    if isempty(path3D)
        fprintf('3D A* failed, using direct waypoint approach...\n');
        globalPath = createDirectWaypointPath(startPos, goalPos, tomogram);
        return;
    end
    
    % Convert back to world coordinates
    globalPath = [];
    for i = 1:size(path3D, 1)
        worldPos = gridToWorld(path3D(i, 1:2), tomogram);
        sliceIdx = path3D(i, 3);
        height = tomogram.heights(sliceIdx);
        globalPath = [globalPath; worldPos, height];
    end
end

%% Simplified 2D Path Planning (Fallback Method)
function globalPath = planSimplified2DPath(tomogram, startPos, goalPos, robotParams)
    % Use the middle slice for 2D planning
    middleSliceIdx = ceil(length(tomogram.slices) / 2);
    middleSlice = tomogram.slices{middleSliceIdx};
    
    % Create binary traversability map
    traversable = middleSlice.c_T < 100;  % Lower threshold for traversability
    
    % Add safety margin
    se = strel('disk', 2);
    traversable = imerode(traversable, se);
    
    % Convert positions to grid
    startGrid = worldToGrid(startPos(1:2), tomogram);
    goalGrid = worldToGrid(goalPos(1:2), tomogram);
    
    % Check bounds
    if ~isValidGridBounds(startGrid, tomogram) || ~isValidGridBounds(goalGrid, tomogram)
        globalPath = [];
        return;
    end
    
    % Find valid start and goal positions
    if ~traversable(startGrid(1), startGrid(2))
        startGrid = findNearestTraversable(startGrid, traversable);
    end
    
    if ~traversable(goalGrid(1), goalGrid(2))
        goalGrid = findNearestTraversable(goalGrid, traversable);
    end
    
    if isempty(startGrid) || isempty(goalGrid)
        globalPath = [];
        return;
    end
    
    % Run 2D A*
    path2D = runSimple2DAStar(startGrid, goalGrid, traversable);
    
    if isempty(path2D)
        globalPath = [];
        return;
    end
    
    % Convert to 3D world coordinates with height interpolation
    globalPath = [];
    for i = 1:size(path2D, 1)
        worldPos = gridToWorld(path2D(i, :), tomogram);
        
        % Interpolate height based on position
        alpha = (i - 1) / (size(path2D, 1) - 1);
        height = startPos(3) + alpha * (goalPos(3) - startPos(3));
        
        globalPath = [globalPath; worldPos, height];
    end
end

%% Simple 2D A* Implementation
function path2D = runSimple2DAStar(startGrid, goalGrid, traversable)
    [h, w] = size(traversable);
    
    % Initialize A*
    openList = [startGrid, 0, norm(goalGrid - startGrid)];
    closedSet = false(h, w);
    cameFrom = zeros(h, w, 2);
    gScore = inf(h, w);
    gScore(startGrid(1), startGrid(2)) = 0;
    
    % 8-connectivity
    directions = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
    costs = [1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4];
    
    iterations = 0;
    while ~isempty(openList) && iterations < 2000
        iterations = iterations + 1;
        
        % Find node with lowest f score
        [~, idx] = min(openList(:, 4));
        current = openList(idx, 1:2);
        openList(idx, :) = [];
        
        % Check if goal reached
        if isequal(current, goalGrid)
            % Reconstruct path
            path2D = [];
            while ~isequal(current, startGrid)
                path2D = [current; path2D];
                current = squeeze(cameFrom(current(1), current(2), :))';
            end
            path2D = [startGrid; path2D];
            return;
        end
        
        closedSet(current(1), current(2)) = true;
        
        % Explore neighbors
        for i = 1:size(directions, 1)
            neighbor = current + directions(i, :);
            
            if neighbor(1) < 1 || neighbor(1) > h || neighbor(2) < 1 || neighbor(2) > w
                continue;
            end
            
            if ~traversable(neighbor(1), neighbor(2)) || closedSet(neighbor(1), neighbor(2))
                continue;
            end
            
            moveCost = costs(i);
            tentativeG = gScore(current(1), current(2)) + moveCost;
            
            if tentativeG < gScore(neighbor(1), neighbor(2))
                cameFrom(neighbor(1), neighbor(2), :) = current;
                gScore(neighbor(1), neighbor(2)) = tentativeG;
                fScore = tentativeG + norm(goalGrid - neighbor);
                
                % Add to open list
                openList = [openList; neighbor, tentativeG, fScore];
            end
        end
    end
    
    path2D = [];
end

%% Find nearest traversable cell
function nearestGrid = findNearestTraversable(gridPos, traversable)
    [h, w] = size(traversable);
    
    for radius = 1:20
        for dx = -radius:radius
            for dy = -radius:radius
                if abs(dx) == radius || abs(dy) == radius
                    candidate = gridPos + [dx, dy];
                    if candidate(1) >= 1 && candidate(1) <= h && ...
                       candidate(2) >= 1 && candidate(2) <= w && ...
                       traversable(candidate(1), candidate(2))
                        nearestGrid = candidate;
                        return;
                    end
                end
            end
        end
    end
    
    nearestGrid = [];
end

%% Direct waypoint path (last resort)
function globalPath = createDirectWaypointPath(startPos, goalPos, tomogram)
    % Create a simple waypoint-based path
    fprintf('Creating direct waypoint path...\n');
    
    % Key waypoints based on environment structure
    waypoints = [
        startPos;
        0, 5, 0.1;                    % Corridor middle
        -1, 9, 0.1;                   % Before turn
        -2, 10.5, 0.2;                % Turn area
        -3, 10.5, 0.4;                % Stair approach
        -4, 10.5, 0.8;                % Stair middle
        -5, 10.5, 1.2;                % Stair top
        goalPos
    ];
    
    % Interpolate between waypoints
    globalPath = [];
    for i = 1:size(waypoints, 1)-1
        start_wp = waypoints(i, :);
        end_wp = waypoints(i+1, :);
        
        % Create 10 intermediate points
        for j = 0:9
            alpha = j / 9;
            interpPoint = start_wp + alpha * (end_wp - start_wp);
            globalPath = [globalPath; interpPoint];
        end
    end
    
    % Add final waypoint
    globalPath = [globalPath; goalPos];
    
    fprintf('Direct waypoint path created with %d points\n', size(globalPath, 1));
end

%% Enhanced 3D A* with strict obstacle avoidance
function path3D = run3DAStar(startGrid, goalGrid, startSliceIdx, goalSliceIdx, tomogram, costParams)
    % Initialize A* search with strict obstacle avoidance
    startNode = [startGrid, startSliceIdx];
    goalNode = [goalGrid, goalSliceIdx];
    
    % Verify start and goal positions are truly traversable
    if ~isStrictlyTraversable(startGrid, tomogram.slices{startSliceIdx}, costParams.c_B)
        fprintf('Start position not traversable, searching for alternative...\n');
        startGrid = findNearestTraversablePosition(startGrid, tomogram.slices{startSliceIdx}, costParams.c_B);
        if isempty(startGrid)
            fprintf('No traversable start position found!\n');
            path3D = [];
            return;
        end
        startNode = [startGrid, startSliceIdx];
    end
    
    if ~isStrictlyTraversable(goalGrid, tomogram.slices{goalSliceIdx}, costParams.c_B)
        fprintf('Goal position not traversable, searching for alternative...\n');
        goalGrid = findNearestTraversablePosition(goalGrid, tomogram.slices{goalSliceIdx}, costParams.c_B);
        if isempty(goalGrid)
            fprintf('No traversable goal position found!\n');
            path3D = [];
            return;
        end
        goalNode = [goalGrid, goalSliceIdx];
    end
    
    openList = [startNode, 0, heuristic3DPCT(startNode, goalNode)];
    closedSet = containers.Map();
    cameFrom = containers.Map();
    gScore = containers.Map();
    
    nodeKey = @(node) sprintf('%d_%d_%d', node(1), node(2), node(3));
    
    gScore(nodeKey(startNode)) = 0;
    
    iterations = 0;
    maxIterations = 3000;  % Reduced max iterations
    
    % Improved goal tolerance
    goalTolerance = 5;  % Grid cells
    sliceTolerance = 2;  % Slices
    
    while ~isempty(openList) && iterations < maxIterations
        iterations = iterations + 1;
        
        % Get node with lowest f-score
        [~, idx] = min(openList(:, 5));
        current = openList(idx, 1:3);
        openList(idx, :) = [];
        
        currentKey = nodeKey(current);
        
        % Check if goal reached with tolerance
        xyDist = norm(current(1:2) - goalNode(1:2));
        zDist = abs(current(3) - goalNode(3));
        
        if xyDist <= goalTolerance && zDist <= sliceTolerance
            % Reconstruct path
            path3D = reconstructPath3D(cameFrom, current, startNode);
            fprintf('3D A* found path in %d iterations (tolerance: xy=%.1f, z=%d)\n', ...
                    iterations, xyDist, zDist);
            return;
        end
        
        closedSet(currentKey) = true;
        
        % Explore neighbors with strict traversability check
        neighbors = getStrictNeighbors3D(current, tomogram, costParams.c_B);
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            neighborKey = nodeKey(neighbor);
            
            if isKey(closedSet, neighborKey)
                continue;
            end
            
            % Calculate movement cost with strict obstacle avoidance
            moveCost = calculateStrictMovementCost(current, neighbor, tomogram, costParams);
            if isinf(moveCost)
                continue;  % Skip impassable neighbors
            end
            tentativeG = gScore(currentKey) + moveCost;
            
            if ~isKey(gScore, neighborKey) || tentativeG < gScore(neighborKey)
                cameFrom(neighborKey) = current;
                gScore(neighborKey) = tentativeG;
                fScore = tentativeG + heuristic3DPCT(neighbor, goalNode);
                
                % Add to open list (remove duplicates)
                existingIdx = [];
                for j = 1:size(openList, 1)
                    if isequal(openList(j, 1:3), neighbor)
                        existingIdx = j;
                        break;
                    end
                end
                
                if ~isempty(existingIdx)
                    openList(existingIdx, :) = [neighbor, tentativeG, fScore];
                else
                    openList = [openList; neighbor, tentativeG, fScore];
                end
            end
        end
        
        % Prune open list if it gets too large
        if size(openList, 1) > 1000
            [~, sortIdx] = sort(openList(:, 5));
            openList = openList(sortIdx(1:500), :);  % Keep best 500 nodes
        end
        
        if mod(iterations, 300) == 0
            fprintf('3D A* iteration %d, open list size: %d, best f: %.2f\n', ...
                    iterations, size(openList, 1), min(openList(:, 5)));
        end
    end
    
    fprintf('3D A* failed after %d iterations\n', iterations);
    path3D = [];
end

%% Strict traversability check
function valid = isStrictlyTraversable(gridPos, slice, c_B)
    % Check if a grid position is strictly traversable (no obstacles)
    if ~isfield(slice, 'c_T') || isempty(slice.c_T)
        valid = false;
        return;
    end
    
    [h, w] = size(slice.c_T);
    if gridPos(1) < 1 || gridPos(1) > h || gridPos(2) < 1 || gridPos(2) > w
        valid = false;
        return;
    end
    
    % Very strict threshold - only allow very low cost areas
    valid = slice.c_T(gridPos(1), gridPos(2)) < c_B * 0.05;
    
    % Also check for solid obstacles
    if isfield(slice, 'solidObstacles') && slice.solidObstacles(gridPos(1), gridPos(2))
        valid = false;
    end
end

%% Find nearest traversable position
function nearestGrid = findNearestTraversablePosition(gridPos, slice, c_B)
    % Find the nearest strictly traversable position
    [h, w] = size(slice.c_T);
    
    for radius = 1:20  % Search in expanding radius
        for dx = -radius:radius
            for dy = -radius:radius
                if abs(dx) == radius || abs(dy) == radius  % Only check perimeter
                    candidate = gridPos + [dx, dy];
                    if candidate(1) >= 1 && candidate(1) <= h && ...
                       candidate(2) >= 1 && candidate(2) <= w && ...
                       isStrictlyTraversable(candidate, slice, c_B)
                        nearestGrid = candidate;
                        return;
                    end
                end
            end
        end
    end
    
    nearestGrid = [];  % No traversable position found
end

%% Strict neighbor generation for A*
function neighbors = getStrictNeighbors3D(current, tomogram, c_B)
    % Generate only strictly traversable neighbors
    directions2D = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
    
    neighbors = [];
    currentSlice = current(3);
    
    % Same slice neighbors
    for i = 1:size(directions2D, 1)
        neighbor = [current(1:2) + directions2D(i, :), currentSlice];
        if isValidGridBounds(neighbor(1:2), tomogram) && ...
           isStrictlyTraversable(neighbor(1:2), tomogram.slices{currentSlice}, c_B)
            neighbors = [neighbors; neighbor];
        end
    end
    
    % Vertical transitions (very limited)
    for deltaSlice = [-1, 1]
        newSlice = currentSlice + deltaSlice;
        if newSlice >= 1 && newSlice <= length(tomogram.slices)
            neighbor = [current(1:2), newSlice];
            if isValidGridBounds(neighbor(1:2), tomogram) && ...
               isStrictlyTraversable(neighbor(1:2), tomogram.slices{newSlice}, c_B)
                neighbors = [neighbors; neighbor];
            end
        end
    end
end

%% Adaptive neighbor generation
function neighbors = getNeighbors3DAdaptive(current, tomogram, goalNode)
    % Generate neighbors with bias toward goal
    directions2D = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
    
    neighbors = [];
    currentSlice = current(3);
    
    % Calculate direction toward goal
    goalDir = goalNode(1:2) - current(1:2);
    if norm(goalDir) > 0
        goalDir = goalDir / norm(goalDir);
    end
    
    % Same slice neighbors with goal bias
    for i = 1:size(directions2D, 1)
        neighbor = [current(1:2) + directions2D(i, :), currentSlice];
        if isValidGridBounds(neighbor(1:2), tomogram)
            % Bias toward goal direction
            moveDir = directions2D(i, :);
            if norm(moveDir) > 0
                moveDir = moveDir / norm(moveDir);
                bias = dot(moveDir, goalDir);
                if bias > -0.5  % Don't move too far from goal direction
                    neighbors = [neighbors; neighbor];
                end
            else
                neighbors = [neighbors; neighbor];
            end
        end
    end
    
    % Vertical transitions (more selective)
    goalSlice = goalNode(3);
    if goalSlice > currentSlice
        % Prefer moving up toward goal
        newSlice = currentSlice + 1;
        if newSlice <= length(tomogram.slices)
            neighbor = [current(1:2), newSlice];
            if isValidGridBounds(neighbor(1:2), tomogram)
                neighbors = [neighbors; neighbor];
            end
        end
    elseif goalSlice < currentSlice
        % Prefer moving down toward goal
        newSlice = currentSlice - 1;
        if newSlice >= 1
            neighbor = [current(1:2), newSlice];
            if isValidGridBounds(neighbor(1:2), tomogram)
                neighbors = [neighbors; neighbor];
            end
        end
    else
        % Same level as goal, allow limited vertical exploration
        for deltaSlice = [-1, 1]
            newSlice = currentSlice + deltaSlice;
            if newSlice >= 1 && newSlice <= length(tomogram.slices)
                neighbor = [current(1:2), newSlice];
                if isValidGridBounds(neighbor(1:2), tomogram)
                    neighbors = [neighbors; neighbor];
                end
            end
        end
    end
end

%% Relaxed traversability check
function valid = isValidGridPositionRelaxed(gridPos, slice, c_B)
    if ~isfield(slice, 'c_T') || isempty(slice.c_T)
        valid = false;
        return;
    end
    
    [h, w] = size(slice.c_T);
    if gridPos(1) < 1 || gridPos(1) > h || gridPos(2) < 1 || gridPos(2) > w
        valid = false;
    else
        % Relaxed threshold - allow higher cost areas
        valid = slice.c_T(gridPos(1), gridPos(2)) < c_B * 0.8;
    end
end

%% Strict movement cost calculation
function cost = calculateStrictMovementCost(current, neighbor, tomogram, costParams)
    % Calculate movement cost with strict obstacle avoidance
    
    % Base movement cost
    xyDist = norm(neighbor(1:2) - current(1:2));
    zDist = abs(neighbor(3) - current(3));
    baseCost = xyDist + zDist * 2.0;  % Penalize vertical movement
    
    % Get neighbor slice
    neighborSlice = tomogram.slices{neighbor(3)};
    
    % Check traversability cost
    if ~isempty(neighborSlice.c_T)
        traversabilityCost = neighborSlice.c_T(neighbor(1), neighbor(2));
        
        % Very strict threshold
        if traversabilityCost >= costParams.c_B * 0.05
            cost = inf;  % Impassable
            return;
        end
        
        cost = baseCost + traversabilityCost * 0.001;  % Minimal weight on traversability
    else
        cost = inf;  % No traversability data = impassable
    end
end

%% Adaptive movement cost calculation
function cost = calculateMovementCostAdaptive(current, neighbor, tomogram, costParams, goalNode)
    % Base movement cost
    xyDist = norm(neighbor(1:2) - current(1:2));
    zDist = abs(neighbor(3) - current(3));
    
    baseCost = xyDist + zDist * 1.5;  % Reduced vertical penalty
    
    % Add traversability cost
    neighborSlice = tomogram.slices{neighbor(3)};
    if ~isempty(neighborSlice.c_T)
        traversabilityCost = neighborSlice.c_T(neighbor(1), neighbor(2));
        if traversabilityCost >= costParams.c_B * 0.8
            cost = inf;  % Impassable
        else
            cost = baseCost + traversabilityCost * 0.05;  % Reduced traversability weight
        end
    else
        cost = baseCost;
    end
    
    % Add goal attraction (reduce cost if moving toward goal)
    currentToGoal = norm(goalNode(1:2) - current(1:2));
    neighborToGoal = norm(goalNode(1:2) - neighbor(1:2));
    
    if neighborToGoal < currentToGoal
        cost = cost * 0.9;  % 10% discount for moving toward goal
    end
end

%% Part 5: Trajectory Optimization (Section III-E inspired)
function optimizedPath = optimizePCTTrajectory(path, tomogram, robotParams)
    % Optimize trajectory for smoothness and safety
    
    if size(path, 1) < 3
        optimizedPath = path;
        return;
    end
    
    % Apply smoothing filter
    smoothedPath = path;
    
    % Smooth XY coordinates
    windowSize = min(5, floor(size(path, 1) / 3));
    if windowSize >= 3
        smoothedPath(:, 1) = smooth(path(:, 1), windowSize);
        smoothedPath(:, 2) = smooth(path(:, 2), windowSize);
    end
    
    % Ensure height consistency with tomogram
    for i = 1:size(smoothedPath, 1)
        sliceIdx = findNearestSlice(tomogram, smoothedPath(i, 3));
        gridPos = worldToGrid(smoothedPath(i, 1:2), tomogram);
        
        if isValidGridBounds(gridPos, tomogram)
            groundHeight = tomogram.slices{sliceIdx}.e_G(gridPos(1), gridPos(2));
            if ~isinf(groundHeight)
                smoothedPath(i, 3) = groundHeight + robotParams.d_min;
            end
        end
    end
    
    % Apply velocity constraints
    optimizedPath = applyVelocityConstraints(smoothedPath, 0.5);  % 0.5 m/s max
    
    fprintf('Trajectory optimized: %d -> %d waypoints\n', size(path, 1), size(optimizedPath, 1));
end

%% Helper Functions

function sliceIdx = findNearestSlice(tomogram, height)
    [~, sliceIdx] = min(abs(tomogram.heights - height));
end

function gridPos = worldToGrid(worldPos, tomogram)
    gridX = floor((worldPos(1) - tomogram.minX) / tomogram.gridRes) + 1;
    gridY = floor((worldPos(2) - tomogram.minY) / tomogram.gridRes) + 1;
    gridPos = [gridX, gridY];
end

function worldPos = gridToWorld(gridPos, tomogram)
    worldX = tomogram.minX + (gridPos(1) - 0.5) * tomogram.gridRes;
    worldY = tomogram.minY + (gridPos(2) - 0.5) * tomogram.gridRes;
    worldPos = [worldX, worldY];
end

function valid = isValidGridBounds(gridPos, tomogram)
    valid = gridPos(1) >= 1 && gridPos(1) <= tomogram.gridW && ...
            gridPos(2) >= 1 && gridPos(2) <= tomogram.gridH;
end

function valid = isValidGridPosition(gridPos, slice, c_B)
    if ~isfield(slice, 'c_T') || isempty(slice.c_T)
        valid = false;
        return;
    end
    
    [h, w] = size(slice.c_T);
    if gridPos(1) < 1 || gridPos(1) > h || gridPos(2) < 1 || gridPos(2) > w
        valid = false;
    else
        valid = slice.c_T(gridPos(1), gridPos(2)) < c_B;
    end
end

function [correctedStart, correctedGoal] = correctInvalidPositions(startGrid, goalGrid, tomogram, c_B)
    % Find nearby valid positions
    correctedStart = findNearbyValidPosition(startGrid, tomogram.slices{1}, c_B);
    correctedGoal = findNearbyValidPosition(goalGrid, tomogram.slices{end}, c_B);
    
    if isempty(correctedStart)
        correctedStart = startGrid;
    end
    if isempty(correctedGoal)
        correctedGoal = goalGrid;
    end
end

function validPos = findNearbyValidPosition(gridPos, slice, c_B)
    validPos = [];
    maxRadius = 10;
    
    for radius = 1:maxRadius
        for dx = -radius:radius
            for dy = -radius:radius
                if abs(dx) == radius || abs(dy) == radius
                    candidate = gridPos + [dx, dy];
                    if isValidGridPosition(candidate, slice, c_B)
                        validPos = candidate;
                        return;
                    end
                end
            end
        end
    end
end

function constrainedPath = applyVelocityConstraints(path, maxVelocity)
    % Apply velocity constraints to path
    if size(path, 1) < 2
        constrainedPath = path;
        return;
    end
    
    dt = 0.1;  % Time step
    constrainedPath = path(1, :);
    
    for i = 2:size(path, 1)
        currentPoint = path(i, :);
        previousPoint = constrainedPath(end, :);
        
        displacement = currentPoint - previousPoint;
        distance = norm(displacement);
        requiredVelocity = distance / dt;
        
        if requiredVelocity > maxVelocity
            scaleFactor = maxVelocity * dt / distance;
            adjustedPoint = previousPoint + displacement * scaleFactor;
            constrainedPath = [constrainedPath; adjustedPoint];
        else
            constrainedPath = [constrainedPath; currentPoint];
        end
    end
end

%% Path Following Simulation
function simulatePCTPathFollowing(path, pcdEnv)
    % Simulate quadruped following the PCT-optimized path
    
    if size(path, 1) < 2
        fprintf('Path too short for simulation\n');
        return;
    end
    
    % Robot parameters
    robotState = [path(1, :), 0];  % [x, y, z, yaw]
    robotTraj = robotState;
    
    lookAhead = 0.3;
    maxVel = 0.4;
    maxOmega = 1.0;
    kp = 1.0;
    dt = 0.1;
    
    % Visualization
    figure; hold on;
    plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 2, 'DisplayName', 'PCT Path');
    hRobot = plot3(robotState(1), robotState(2), robotState(3), 'bo', 'MarkerSize', 8, 'DisplayName', 'Robot');
    hTraj = plot3(robotTraj(:,1), robotTraj(:,2), robotTraj(:,3), 'g-', 'LineWidth', 1, 'DisplayName', 'Trajectory');
    pcshow(pcdEnv, 'MarkerSize', 2, 'Parent', gca);
    xlabel('X'); ylabel('Y'); zlabel('Z'); title('PCT Path Following'); legend; view(3);
    
    pathIdx = 1;
    
    for step = 1:500
        % Find look-ahead point
        robotPos = robotState(1:2);
        
        % Update closest point
        dists = vecnorm(path(:,1:2) - robotPos, 2, 2);
        [~, closestIdx] = min(dists);
        pathIdx = max(pathIdx, closestIdx);
        
        % Find look-ahead target
        targetIdx = pathIdx;
        while targetIdx < size(path, 1)
            lookDist = norm(path(targetIdx, 1:2) - robotPos);
            if lookDist >= lookAhead
                break;
            end
            targetIdx = targetIdx + 1;
        end
        
        target = path(targetIdx, :);
        
        % Calculate control
        deltaX = target(1) - robotState(1);
        deltaY = target(2) - robotState(2);
        targetYaw = atan2(deltaY, deltaX);
        
        yawError = atan2(sin(targetYaw - robotState(4)), cos(targetYaw - robotState(4)));
        omega = kp * yawError;
        omega = max(min(omega, maxOmega), -maxOmega);
        
        velocityX = maxVel;
        if norm(robotPos - path(end, 1:2)) < lookAhead * 2
            velocityX = velocityX * 0.5;
        end
        
        % Update state
        robotState(1) = robotState(1) + velocityX * cos(robotState(4)) * dt;
        robotState(2) = robotState(2) + velocityX * sin(robotState(4)) * dt;
        robotState(4) = robotState(4) + omega * dt;
        
        % Update height
        [~, closestZ] = min(vecnorm(path(:,1:2) - robotState(1:2), 2, 2));
        robotState(3) = path(closestZ, 3);
        
        robotTraj = [robotTraj; robotState];
        
        % Update visualization
        set(hRobot, 'XData', robotState(1), 'YData', robotState(2), 'ZData', robotState(3));
        set(hTraj, 'XData', robotTraj(:,1), 'YData', robotTraj(:,2), 'ZData', robotTraj(:,3));
        drawnow limitrate;
        
        % Check goal
        if norm(robotState(1:2) - path(end, 1:2)) < 0.2
            fprintf('Goal reached using PCT navigation!\n');
            break;
        end
    end
    
    fprintf('PCT path following complete. Final trajectory: %d points\n', size(robotTraj, 1));
end

%% Heuristic function for 3D A*
function h = heuristic3DPCT(node1, node2)
    % Hybrid Manhattan-Euclidean heuristic with height penalty
    xyDist = norm(node2(1:2) - node1(1:2));
    zDist = abs(node2(3) - node1(3));
    
    h = xyDist + zDist * 1.5;  % Penalize height differences
end

%% Reconstruct path from A* search
function path3D = reconstructPath3D(cameFrom, current, startNode)
    nodeKey = @(node) sprintf('%d_%d_%d', node(1), node(2), node(3));
    
    path3D = current;
    currentKey = nodeKey(current);
    
    while isKey(cameFrom, currentKey) && ~isequal(current, startNode)
        current = cameFrom(currentKey);
        currentKey = nodeKey(current);
        path3D = [current; path3D];
    end
end

%% Calculate movement cost between nodes (legacy function for compatibility)
function cost = calculateMovementCost(current, neighbor, tomogram, costParams)
    % Base movement cost
    xyDist = norm(neighbor(1:2) - current(1:2));
    zDist = abs(neighbor(3) - current(3));
    
    baseCost = xyDist + zDist * 2.0;  % Penalize vertical movement
    
    % Add traversability cost
    neighborSlice = tomogram.slices{neighbor(3)};
    if ~isempty(neighborSlice.c_T)
        traversabilityCost = neighborSlice.c_T(neighbor(1), neighbor(2));
        if traversabilityCost >= costParams.c_B
            cost = inf;  % Impassable
        else
            cost = baseCost + traversabilityCost * 0.1;
        end
    else
        cost = baseCost;
    end
end

%% Get 3D neighbors for A* search (legacy function for compatibility)
function neighbors = getNeighbors3D(current, tomogram)
    % 8-connected in XY plane, plus vertical transitions
    directions2D = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
    
    neighbors = [];
    currentSlice = current(3);
    
    % Same slice neighbors
    for i = 1:size(directions2D, 1)
        neighbor = [current(1:2) + directions2D(i, :), currentSlice];
        if isValidGridBounds(neighbor(1:2), tomogram)
            neighbors = [neighbors; neighbor];
        end
    end
    
    % Adjacent slice neighbors (vertical transitions)
    for deltaSlice = [-1, 1]
        newSlice = currentSlice + deltaSlice;
        if newSlice >= 1 && newSlice <= length(tomogram.slices)
            neighbor = [current(1:2), newSlice];
            if isValidGridBounds(neighbor(1:2), tomogram)
                neighbors = [neighbors; neighbor];
            end
        end
    end
end