%% Analyze stair traversability
% This script combines the simple stair scene with our traversability analysis

clear; clc;

% First generate the stair scene
run('simple_stair_perception.m');

% Initialize the tomogram processor with fine resolution
ds = 0.2;  % 15cm slice interval (smaller than step height for better coverage)
rg = 0.10;   % 10cm grid resolution
processor = TomogramProcessor(ds, rg);

% Adjust robot parameters for quadruped stair climbing
processor.robotParams.d_min = 0.10;    % Minimum clearance (reduced)
processor.robotParams.d_ref = 0.30;    % Reference height (reduced) 
processor.robotParams.d_sm = 0.25;     % Safety margin

% Adjust cost parameters for more permissive traversability
processor.costParams.c_B = 130;        % Slightly higher barrier cost to provide buffer
processor.costParams.theta_s = 1.2;    % Higher slope tolerance
processor.costParams.theta_p = 0.05;   % More permissive safe neighbor ratio

% Load the point cloud
processor.loadPointCloud(pcd.Location);

% Override support distance threshold for stair climbing
% Allow larger vertical gaps for quadruped robots
processor.ds = 0.20;  % Ensure ds is set correctly

% CRITICAL: Support distance threshold has been increased in TomogramProcessor.m
% From 1.5*ds (0.30m) to 4.0*ds (0.80m) for stair climbing capability
fprintf('Applying stair-specific traversability adjustments...\n');
fprintf('Modified support distance threshold: %.2f m (4.0 * %.2f)\n', ...
    4.0*processor.ds, processor.ds);

% Process tomograms
fprintf('\n=== Step 1: Processing Tomograms ===\n');
processor.processTomograms();

% Diagnostic: Analyze why costs are so high
fprintf('\n=== COST ANALYSIS DIAGNOSTICS ===\n');
for k = 1:min(3, length(processor.slices))  % Check first 3 slices
    slice = processor.slices{k};
    fprintf('Slice %d (z=%.2fm):\n', k, slice.z);
    
    % Find a typical high-cost cell for analysis
    high_cost_mask = slice.cT == processor.costParams.c_B;
    [I, J] = find(high_cost_mask, 1, 'first');
    
    if ~isempty(I)
        i = I(1); j = J(1);
        fprintf('  Analyzing high-cost cell [%d,%d]:\n', i, j);
        
        % Check basic data
        fprintf('    Ground height: %.3f\n', slice.eG(i,j));
        fprintf('    Ceiling height: %.3f\n', slice.eC(i,j));
        fprintf('    Clearance: %.3f\n', slice.eC(i,j) - slice.eG(i,j));
        fprintf('    Support distance: %.3f\n', slice.z - slice.eG(i,j));
        
        % Check gradients if available
        if ~isnan(slice.eG(i,j))
            [gx, gy] = processor.calculateGradients(slice.eG);
            if i <= size(gx,1) && j <= size(gx,2)
                grad_mag = sqrt(gx(i,j)^2 + gy(i,j)^2);
                max_grad = max(abs(gx(i,j)), abs(gy(i,j)));
                fprintf('    Gradient magnitude: %.3f (threshold: %.1f)\n', grad_mag, processor.costParams.theta_s);
                fprintf('    Max gradient: %.3f (boundary threshold: %.1f)\n', max_grad, processor.costParams.theta_b);
                
                % Check neighbor analysis
                if grad_mag >= processor.costParams.theta_s && i > 1 && i < size(slice.eG,1) && j > 1 && j < size(slice.eG,2)
                    patch = sqrt(gx(i-1:i+1, j-1:j+1).^2 + gy(i-1:i+1, j-1:j+1).^2);
                    valid_patch = ~isnan(patch);
                    if any(valid_patch(:))
                        ps = sum(patch(:) < processor.costParams.theta_s & valid_patch(:)) / sum(valid_patch(:));
                        fprintf('    Safe neighbor ratio (ps): %.3f (threshold: %.2f)\n', ps, processor.costParams.theta_p);
                        
                        if max_grad > processor.costParams.theta_b
                            fprintf('    -> HIGH COST REASON: Max gradient exceeds boundary threshold\n');
                        elseif ps <= processor.costParams.theta_p
                            fprintf('    -> HIGH COST REASON: Insufficient safe neighbors\n');
                        end
                    else
                        fprintf('    -> HIGH COST REASON: No valid neighbors for analysis\n');
                    end
                else
                    fprintf('    -> Gradient below threshold, should have low cost\n');
                end
            end
        end
    else
        fprintf('  No high-cost cells found in this slice\n');
    end
    fprintf('\n');
end

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
% Use positions within valid slice ranges
% First slice covers [0.05m, 0.25m], so use 0.10m (middle of range)
startPos = [0.0, 0.05, 0.10];  % Within Slice 1 range
endPos = [0.0, 4.05, 0.10];    % Within Slice 1 range

fprintf('Planning path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]\n', ...
    startPos(1), startPos(2), startPos(3), endPos(1), endPos(2), endPos(3));

% Plan initial path with detailed debugging
fprintf('\n=== DETAILED PATH PLANNING DEBUG ===\n');
fprintf('Start position: [%.3f, %.3f, %.3f]\n', startPos(1), startPos(2), startPos(3));
fprintf('End position: [%.3f, %.3f, %.3f]\n', endPos(1), endPos(2), endPos(3));

% Test slice finding
startSlice = processor.findSliceContaining(startPos(3));
endSlice = processor.findSliceContaining(endPos(3));
fprintf('Start slice: %d\n', startSlice);
fprintf('End slice: %d\n', endSlice);

if startSlice == 0 || endSlice == 0
    fprintf('ERROR: Start or end position not in any slice!\n');
    fprintf('Available slice heights:\n');
    for i = 1:length(processor.slices)
        slice = processor.slices{i};
        range_min = slice.z - processor.ds/2;
        range_max = slice.z + processor.ds/2;
        fprintf('  Slice %d: z=%.2fm (range: %.3f to %.3f)\n', i, slice.z, range_min, range_max);
    end
    error('Start or end position outside slice ranges!');
end

% Test grid conversion
startI = max(1, min(processor.gridSize(1), ceil((startPos(2) - processor.minY) / processor.rg)));
startJ = max(1, min(processor.gridSize(2), ceil((startPos(1) - processor.minX) / processor.rg)));
endI = max(1, min(processor.gridSize(1), ceil((endPos(2) - processor.minY) / processor.rg)));
endJ = max(1, min(processor.gridSize(2), ceil((endPos(1) - processor.minX) / processor.rg)));

fprintf('Start grid: [%d, %d, %d]\n', startI, startJ, startSlice);
fprintf('End grid: [%d, %d, %d]\n', endI, endJ, endSlice);

% Test traversability
startTraversable = processor.isTraversable(startI, startJ, startSlice);
endTraversable = processor.isTraversable(endI, endJ, endSlice);
fprintf('Start traversable: %s\n', mat2str(startTraversable));
fprintf('End traversable: %s\n', mat2str(endTraversable));

if ~startTraversable
    fprintf('ERROR: Start position not traversable!\n');
    processor.analyzeGridPosition(startI, startJ, startSlice, 'START');
    error('Start position blocked!');
end

if ~endTraversable
    fprintf('ERROR: End position not traversable!\n');
    processor.analyzeGridPosition(endI, endJ, endSlice, 'END');
    error('End position blocked!');
end

% Test neighbor generation
fprintf('Testing neighbor generation for start position...\n');
startNode = [startI, startJ, startSlice];
neighbors = processor.getNeighborsWithGateways(startNode);
fprintf('Start position has %d neighbors:\n', size(neighbors, 1));
for i = 1:min(5, size(neighbors, 1))  % Show first 5 neighbors
    neighbor = neighbors(i, :);
    cost = processor.calculateNodeCost(startNode, neighbor);
    fprintf('  Neighbor [%d,%d,%d]: cost=%.3f\n', neighbor(1), neighbor(2), neighbor(3), cost);
end

rawPath = processor.planPath(startPos, endPos);

if isempty(rawPath)
    fprintf('\n=== PATH PLANNING FAILED ===\n');
    fprintf('Running detailed scenario analysis...\n');
    processor.analyzeStairScenario(startPos, endPos);
    
    fprintf('\n=== ADDITIONAL DEBUG INFO ===\n');
    fprintf('Total slices: %d\n', length(processor.slices));
    for i = 1:length(processor.slices)
        slice = processor.slices{i};
        traversableCount = sum(slice.cT(:) < processor.costParams.c_B & ~isnan(slice.cT(:)));
        totalCells = numel(slice.cT);
        fprintf('Slice %d: %d/%d traversable (%.1f%%)\n', i, traversableCount, totalCells, 100*traversableCount/totalCells);
    end
    
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