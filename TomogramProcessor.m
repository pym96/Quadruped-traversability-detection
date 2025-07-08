classdef TomogramProcessor < handle
    properties
        pointCloud   % Raw point cloud data
        ds          % Slice interval (m)
        rg          % Grid resolution (m)
        slices      % Cell array of tomogram slices
        zMin        % Minimum z value
        zMax        % Maximum z value
        N           % Number of slices
        gridSize    % Size of grid [rows, cols]
        costParams  % Cost parameters for traversability
        robotParams % Robot parameters
        
        % Add coordinate system bounds
        minX        % Minimum X coordinate
        maxX        % Maximum X coordinate
        minY        % Minimum Y coordinate
        maxY        % Maximum Y coordinate
        minZ        % Minimum Z coordinate
        maxZ        % Maximum Z coordinate
    end
    
    methods
        function obj = TomogramProcessor(ds, rg)
            % Constructor
            if nargin < 2
                obj.ds = 0.5;  % Default 20cm slice interval
                obj.rg = 0.2;  % Default 10cm grid resolution
            else
                obj.ds = ds;
                obj.rg = rg;
            end
            
            % Initialize parameters for traversability estimation
            obj.robotParams.d_min = 0.15;    % Minimum robot height (reduced for quadruped)
            obj.robotParams.d_ref = 0.40;    % Reference (normal) robot height (reduced)
            % Safety margin for vertical transitions
            obj.robotParams.d_sm = 0.3;      % Increased for better stair climbing
            % Maximum acceptable ground slope (same as theta_s threshold)
            obj.robotParams.theta_max = 1.0; % Increased slope tolerance
            
            obj.costParams.c_B = 150;        % Barrier/obstacle cost (higher for powerful quadruped)
            obj.costParams.alpha_d = 0.2;    % Height adjustment cost factor (reduced)
            obj.costParams.theta_b = 15;     % Obstacle boundary threshold (increased)
            obj.costParams.theta_s = 1.0;    % Flat surface threshold (increased for slope tolerance)
            obj.costParams.theta_p = 0.05;    % Safe neighbor ratio threshold (reduced for more permissive)
            obj.costParams.alpha_s = 0.05;   % Surface cost factor (reduced)
            obj.costParams.alpha_b = 0.05;   % Boundary cost factor (reduced)
            obj.costParams.r_g = 0.15;       % Grid resolution
            obj.costParams.d_inf = 0.2;      % Inflation distance
            obj.costParams.d_sm = 0.3;       % Safety margin (matched with robot params)
            
            % Initialize coordinate bounds (will be updated in processPointCloud)
            obj.minX = inf;
            obj.maxX = -inf;
            obj.minY = inf;
            obj.maxY = -inf;
            obj.minZ = inf;
            obj.maxZ = -inf;
        end
        
        function obj = loadPointCloud(obj, points)
            % Load point cloud data and initialize parameters
            obj.pointCloud = points;
            rawZmin = min(points(:,3));
            obj.zMin = floor(rawZmin / obj.ds) * obj.ds - 0.10;  % align to nearest slice plane
            obj.zMax = max(points(:,3));
            obj.N = ceil((obj.zMax - obj.zMin) / obj.ds);  % number of slices
            
            % Calculate grid size based on point cloud bounds
            xMax = max(points(:,1)); xMin = min(points(:,1));
            yMax = max(points(:,2)); yMin = min(points(:,2));
            obj.gridSize = [ceil((yMax-yMin)/obj.rg), ceil((xMax-xMin)/obj.rg)];
            
            % Update coordinate bounds
            obj.minX = min(points(:,1));
            obj.maxX = max(points(:,1));
            obj.minY = min(points(:,2));
            obj.maxY = max(points(:,2));
            obj.minZ = min(points(:,3));
            obj.maxZ = max(points(:,3));
            
            % Initialize slices array with struct for each slice
            obj.slices = cell(obj.N + 1, 1);
            for k = 0:obj.N
                obj.slices{k + 1} = struct(...
                    'k', k, ...
                    'z', obj.zMin + k * obj.ds, ...
                    'eG', nan(obj.gridSize), ...
                    'eC', nan(obj.gridSize), ...
                    'cT', nan(obj.gridSize), ...
                    'dI', nan(obj.gridSize), ...
                    'gateways', [] ...
                );
            end
        end
        
        function obj = processTomograms(obj)
            % Process point cloud into tomogram slices
            fprintf('\nProcessing Tomograms:\n');
            fprintf('Grid resolution: %.3f m\n', obj.rg);
            fprintf('Slice interval: %.3f m\n', obj.ds);
            fprintf('Robot parameters:\n');
            fprintf('  Minimum height (d_min): %.2f m\n', obj.robotParams.d_min);
            fprintf('  Reference height (d_ref): %.2f m\n', obj.robotParams.d_ref);
            fprintf('Cost parameters:\n');
            fprintf('  Barrier cost (c_B): %.1f\n', obj.costParams.c_B);
            fprintf('  Boundary threshold (theta_b): %.1f\n', obj.costParams.theta_b);
            fprintf('  Surface threshold (theta_s): %.1f\n', obj.costParams.theta_s);
            fprintf('  Safe neighbor ratio (theta_p): %.2f\n', obj.costParams.theta_p);
            fprintf('\nProcessing slices...\n');
            
            % Initialize all slices
            for k = 0:obj.N
                obj.slices{k+1}.eG = -inf(obj.gridSize);  % Ground height map (initialized to -inf)
                obj.slices{k+1}.eC = inf(obj.gridSize);   % Ceiling height map (initialized to inf)
            end
            
            % Process each point
            for u = 1:size(obj.pointCloud, 1)
                x = obj.pointCloud(u,1);
                y = obj.pointCloud(u,2);
                z = obj.pointCloud(u,3);
                
                % Skip if NaN
                if isnan(x) || isnan(y) || isnan(z)
                    continue;
                end
                
                % Convert to grid coordinates
                i = max(1, min(obj.gridSize(1), ceil((y-min(obj.pointCloud(:,2)))/obj.rg)));
                j = max(1, min(obj.gridSize(2), ceil((x-min(obj.pointCloud(:,1)))/obj.rg)));
                
                % For each slice
                for k = 0:obj.N
                    slice = obj.slices{k+1};
                    slice_z = obj.zMin + k * obj.ds;
                    
                    % CRITICAL FIX: Only assign ground points that are reasonably close to slice height
                    % This prevents distant ground points from creating "floating" traversable areas
                    max_ground_distance = 1.0 * obj.ds;  % 1.0 * ds = 0.2m maximum distance
                    
                    % If point is below or at slice height AND within reasonable distance
                    if z <= slice_z && (slice_z - z) <= max_ground_distance
                        slice.eG(i,j) = max(z, slice.eG(i,j));
                    % If point is above slice height, update ceiling (min)
                    elseif z > slice_z
                        slice.eC(i,j) = min(z, slice.eC(i,j));
                    end
                    
                    obj.slices{k+1} = slice;
                end
            end
            
            % Post-process: convert inf/-inf to NaN
            for k = 0:obj.N
                slice = obj.slices{k+1};
                slice.eG(isinf(slice.eG)) = nan;
                slice.eC(isinf(slice.eC)) = nan;
                % If either ground or ceiling invalid, mark both invalid (no complete pair)
                invalidMask = isnan(slice.eG) | isnan(slice.eC);
                slice.eG(invalidMask) = nan;
                slice.eC(invalidMask) = nan;
                obj.slices{k+1} = slice;
            end
            
            % Print statistics for each slice
            for k = 0:obj.N
                slice = obj.slices{k+1};
                plane_z = obj.zMin + k * obj.ds;
                
                fprintf('\n=== Slice %d/%d (z=%.2fm) ===\n', k, obj.N, plane_z);
                fprintf('Point Distribution:\n');
                fprintf('  Valid ground cells: %d\n', sum(~isnan(slice.eG(:))));
                fprintf('  Valid ceiling cells: %d\n', sum(~isnan(slice.eC(:))));
                
                % Calculate height interval
                slice.dI = slice.eC - slice.eG;
                
                % Print height interval statistics
                fprintf('Height Interval (dI):\n');
                validDI = slice.dI(~isnan(slice.dI));
                if ~isempty(validDI)
                    fprintf('  Min: %.2fm, Max: %.2fm, Mean: %.2fm\n', ...
                        min(validDI(:)), max(validDI(:)), mean(validDI(:)));
                    fprintf('  Valid measurements: %d\n', numel(validDI));
                    
                    % Additional height distribution analysis
                    edges = 0:0.05:1.0;  % 5cm bins up to 1m
                    histogram = histcounts(validDI, edges);
                    fprintf('  Height distribution (5cm bins):\n');
                    for i = 1:length(histogram)
                        if histogram(i) > 0
                            fprintf('    %.2f-%.2fm: %d cells\n', edges(i), edges(i+1), histogram(i));
                        end
                    end
                else
                    fprintf('  No valid measurements\n');
                end
                
                % Print ground height distribution
                validG = slice.eG(~isnan(slice.eG));
                if ~isempty(validG)
                    fprintf('Ground heights:\n');
                    fprintf('  Min: %.2fm, Max: %.2fm, Mean: %.2fm\n', ...
                        min(validG(:)), max(validG(:)), mean(validG(:)));
                end
                
                % Print ceiling height distribution
                validC = slice.eC(~isnan(slice.eC));
                if ~isempty(validC)
                    fprintf('Ceiling heights:\n');
                    fprintf('  Min: %.2fm, Max: %.2fm, Mean: %.2fm\n', ...
                        min(validC(:)), max(validC(:)), mean(validC(:)));
                end
                
                % Calculate traversability cost and intermediate maps
                [slice.cT, cI, cG, cInit, gx, gy, mxy] = obj.calculateTraversabilityCost(slice);
                % Save intermediate cost map for visualization if desired
                slice.cInit = cInit;
                slice.cI = cI;     % optional, could be used later
                slice.cG = cG;
                
                % Log statistics for debug
                fprintf('Cost Components (slice %d):\n', k);
                statsFun = @(M) sprintf('min %.1f, max %.1f, mean %.1f', ...
                    min(M(:)), max(M(:)), mean(M(:)));
                validMask = ~isnan(cI);
                if any(validMask(:))
                    fprintf('  cI   -> %s\n', statsFun(cI(validMask)));
                end
                validMask = ~isnan(cG);
                if any(validMask(:))
                    fprintf('  cG   -> %s\n', statsFun(cG(validMask)));
                end
                validMask = ~isnan(cInit);
                if any(validMask(:))
                    fprintf('  cInit-> %s\n', statsFun(cInit(validMask)));
                end
                
                % Log gradient stats
                fprintf('  gx   -> %s\n', statsFun(gx(~isnan(gx))));
                fprintf('  gy   -> %s\n', statsFun(gy(~isnan(gy))));
                fprintf('  mxy  -> %s\n', statsFun(mxy(~isnan(mxy))));
                
                % Print traversability statistics
                validCost = slice.cT(~isnan(slice.cT));
                if ~isempty(validCost)
                    fprintf('Traversability cost:\n');
                    fprintf('  Min: %.2f, Max: %.2f, Mean: %.2f\n', ...
                        min(validCost(:)), max(validCost(:)), mean(validCost(:)));
                    fprintf('  Blocked cells (cost >= %.1f): %d\n', ...
                        obj.costParams.c_B, sum(validCost >= obj.costParams.c_B));
                    fprintf('  Traversable cells: %d\n', ...
                        sum(validCost < obj.costParams.c_B));
                end
                
                obj.slices{k+1} = slice;
                
                % Visualize this slice
                obj.visualizeSlice(k);
            end
            
            % Remove invalid slices
            obj.removeInvalidSlices();
            
            % Plot cost trends
            obj.plotCostTrends();
        end
        
        function visualizeSlice(obj, k)
            % Create a new figure for this slice
            figure('Name', sprintf('Slice %d Analysis', k));
            
            % Get the slice data
            slice = obj.slices{k+1};
            
            % Subplot 1: Ground height map
            subplot(2,2,1);
            imagesc(slice.eG);
            colorbar;
            title(sprintf('Ground Height Map (z=%.2fm)', obj.zMin + k * obj.ds));
            axis equal tight;
            
            % Subplot 2: Ceiling height map
            subplot(2,2,2);
            imagesc(slice.eC);
            colorbar;
            title('Ceiling Height Map');
            axis equal tight;
            
            % Subplot 3: Height interval
            subplot(2,2,3);
            imagesc(slice.dI);
            colorbar;
            title('Height Interval (dI)');
            axis equal tight;
            
            % Subplot 4: Traversability cost
            subplot(2,2,4);
            imagesc(slice.cT);
            colorbar;
            title('Traversability Cost');
            axis equal tight;
            
            % Add overall title
            sgtitle(sprintf('Slice %d Analysis (z=%.2fm)', k, obj.zMin + k * obj.ds));
        end
        
        function obj = removeInvalidSlices(obj)
            % Remove slices that have no valid grids
            validSlices = {};
            validIdx = 1;
            
            for k = 1:length(obj.slices)
                slice = obj.slices{k};
                
                % Find grids where both ground and cost are valid
                validGroundMask = ~isnan(slice.eG);
                validCostMask = ~isnan(slice.cT);
                uniqueValidGrids = validGroundMask & validCostMask;
                
                % If there are valid grids, keep the slice
                if sum(uniqueValidGrids(:)) > 0
                    validSlices{validIdx} = slice;
                    validIdx = validIdx + 1;
                end
            end
            
            % Update slices with only valid ones
            obj.slices = validSlices;
        end
        
        function [cT, cI, cG, cInit, gx, gy, mxy] = calculateTraversabilityCost(obj, slice)
            % Calculate traversability cost for a slice
            [rows, cols] = size(slice.eG);
            cT = zeros(rows, cols);
            cI = nan(size(slice.dI));

            % Invalid or too low clearance -> barrier cost
            maskInvalid = isnan(slice.dI) | slice.dI < obj.robotParams.d_min;
            cI(maskInvalid) = obj.costParams.c_B;

            % Adjustable region between d_min and d_ref
            maskAdjust = slice.dI >= obj.robotParams.d_min & slice.dI <= obj.robotParams.d_ref;
            cI(maskAdjust) = max(0, obj.costParams.alpha_d * (obj.robotParams.d_ref - slice.dI(maskAdjust)));

            [cG, gx, gy, mxy] = obj.calculateGroundCost(slice.eG);

            maskHigh = slice.dI > obj.robotParams.d_ref;
            cI(maskHigh) = 0;                 % 净空充足 → 0
            % Fuse costs: if either component hits barrier, keep barrier;
            % otherwise take weighted sum favouring ground cost.
            cInit = max(cI, 0.4*cI + 0.6*cG);
            cT = min(obj.costParams.c_B, cInit);

            % -----------------------------------------------------------------
            % Additional invalidation: if the ground that supports this slice
            % is farther than half a slice below the current plane, the robot
            % cannot actually stand here.  Mark these cells invalid (NaN) so
            % they will neither be considered traversable nor be rendered
            % as blue patches in the visualization.
            % -----------------------------------------------------------------
            supportMap = slice.z - slice.eG;          % vertical distance to ground
            farSupportMask = supportMap > 2 * obj.ds;    % increased for stair climbing (0.6m)
            cT(farSupportMask) = NaN;
            cI(farSupportMask) = NaN;
            cG(farSupportMask) = NaN;
            cInit(farSupportMask) = NaN;
        end
        
        function [cG, gx, gy, mxy] = calculateGroundCost(obj, eG)
            % Calculate ground-based cost
            [rows, cols] = size(eG);
            cG = nan(rows, cols);
            
            % Calculate gradients
            [gx, gy] = obj.calculateGradients(eG);
            mxy = max(abs(gx), abs(gy));
            mgrad = sqrt(gx.^2 + gy.^2);
            
            % Initialize all costs to zero, set barriers only for invalid cells
            cG = zeros(rows, cols);
            cG(isnan(eG)) = obj.costParams.c_B;
            
            % Process ALL valid cells, not just interior ones
            for i = 1:rows
                for j = 1:cols
                    if ~isnan(eG(i,j))
                        % Check if we have valid gradient at this position
                        if mgrad(i,j) == 0 && (i == 1 || i == rows || j == 1 || j == cols)
                            % Boundary cell with no gradient - use moderate cost
                            cG(i,j) = obj.costParams.alpha_s * 0.1; % Small constant cost for boundaries
                        elseif mgrad(i,j) < obj.costParams.theta_s
                            cG(i,j) = obj.costParams.alpha_s * (mgrad(i,j) / obj.costParams.theta_s^2);
                        else
                            % For neighbor analysis, ensure we don't go out of bounds
                            i_min = max(1, i-1);
                            i_max = min(rows, i+1);
                            j_min = max(1, j-1);
                            j_max = min(cols, j+1);
                            
                            patch = mgrad(i_min:i_max, j_min:j_max);
                            valid_patch = ~isnan(patch);
                            if any(valid_patch(:))
                                ps = sum(patch(:) < obj.costParams.theta_s & valid_patch(:)) / sum(valid_patch(:));
                                if mxy(i,j) > obj.costParams.theta_b
                                    cG(i,j) = obj.costParams.c_B;
                                elseif ps > obj.costParams.theta_p
                                    cG(i,j) = obj.costParams.alpha_b * (mxy(i,j) / obj.costParams.theta_b^2);
                                else
                                    cG(i,j) = obj.costParams.c_B;
                                end
                            else
                                % No valid neighbors - use moderate cost instead of barrier
                                cG(i,j) = obj.costParams.alpha_s * 0.5;
                            end
                        end
                    end
                end
            end
        end
        
        function [gx, gy] = calculateGradients(obj, eG)
            % Calculate gradients using central difference
            [rows, cols] = size(eG);
            gx = zeros(size(eG));
            gy = zeros(size(eG));
            
            % X direction gradient
            for i = 1:rows
                for j = 2:cols-1
                    if ~isnan(eG(i,j+1)) && ~isnan(eG(i,j-1))
                        gx(i,j) = (eG(i,j+1) - eG(i,j-1)) / (2*obj.rg);
                    end
                end
            end
            
            % Y direction gradient
            for j = 1:cols
                for i = 2:rows-1
                    if ~isnan(eG(i+1,j)) && ~isnan(eG(i-1,j))
                        gy(i,j) = (eG(i+1,j) - eG(i-1,j)) / (2*obj.rg);
                    end
                end
            end
        end
        
        function obj = identifyGateways(obj)
            % 识别相邻切片之间的连接
            for k = 1:obj.N
                slice_lower = obj.slices{k};
                slice_upper = obj.slices{k+1};
                
                % 找到在两个切片中都可通行的单元格
                traversable_lower = slice_lower.cT < obj.costParams.c_B;
                traversable_upper = slice_upper.cT < obj.costParams.c_B;
                [i, j] = find(traversable_lower & traversable_upper);
                
                % 存储gateway信息
                if ~isempty(i)
                    gateways = struct('i', num2cell(i), 'j', num2cell(j), ...
                        'lower_k', k, 'upper_k', k+1);
                    slice_lower.gateways = [slice_lower.gateways; gateways];
                    slice_upper.gateways = [slice_upper.gateways; gateways];
                end
            end
        end
        
        function obj = simplifyTomograms(obj)
            % Simplify tomogram slices according to PCT paper Algorithm 1 (lines 9-11)
            % The principle: If Mk ⊆ (Mk-1 ∪ Mk+1), then slice Sk is redundant
            % where Mk denotes the set containing all traversable grids in slice Sk
            
            fprintf('\n=== Tomogram Simplification ===\n');
            fprintf('Following PCT paper Algorithm 1 principle:\n');
            fprintf('Remove slice k if Mk ⊆ (Mk-1 ∪ Mk+1)\n');
            
            original_count = length(obj.slices);
            fprintf('Original slices: %d\n', original_count);
            
            % Always keep first and last slices
            if original_count <= 2
                fprintf('Too few slices to simplify.\n');
                return;
            end
            
            valid_slices = true(original_count, 1);
            removed_count = 0;
            
            % Check each intermediate slice (skip first and last)
            k = 2;  % Start from second slice (index 2)
            while k <= length(obj.slices) - 1
                if k > length(obj.slices) - 1
                    break;  % Safety check
                end
                
                slice_k = obj.slices{k};
                slice_prev = obj.slices{k-1};
                slice_next = obj.slices{k+1};
                
                % Define Mk: set of traversable grids in slice k
                % A grid (i,j) is traversable if cT < c_B
                Mk = (slice_k.cT < obj.costParams.c_B) & ~isnan(slice_k.cT);
                Mk_prev = (slice_prev.cT < obj.costParams.c_B) & ~isnan(slice_prev.cT);
                Mk_next = (slice_next.cT < obj.costParams.c_B) & ~isnan(slice_next.cT);
                
                % Check if Mk ⊆ (Mk-1 ∪ Mk+1)
                % This means every traversable grid in k must also be traversable 
                % in either k-1 or k+1 (or both)
                union_prev_next = Mk_prev | Mk_next;
                is_subset = all(Mk(:) <= union_prev_next(:));  % Mk ⊆ union
                
                % Count grids for reporting
                count_k = sum(Mk(:));
                count_prev = sum(Mk_prev(:));
                count_next = sum(Mk_next(:));
                count_union = sum(union_prev_next(:));
                
                if is_subset
                    % Slice k is redundant - can be omitted
                    fprintf('Slice %d (z=%.2fm): Mk ⊆ (Mk-1 ∪ Mk+1) -> REMOVED\n', ...
                        k-1, slice_k.z);
                    fprintf('  |Mk|=%d, |Mk-1|=%d, |Mk+1|=%d, |Mk-1∪Mk+1|=%d\n', ...
                        count_k, count_prev, count_next, count_union);
                    
                    % Remove this slice
                    obj.slices(k) = [];
                    removed_count = removed_count + 1;
                    
                    % After removal, check if the next slice (now at position k) 
                    % becomes redundant relative to the NEW neighbors
                    % Don't increment k, re-check current position
                    
                else
                    % Slice k is preserved
                    fprintf('Slice %d (z=%.2fm): Mk ⊄ (Mk-1 ∪ Mk+1) -> KEPT\n', ...
                        k-1, slice_k.z);
                    fprintf('  |Mk|=%d, |Mk-1|=%d, |Mk+1|=%d, |Mk-1∪Mk+1|=%d\n', ...
                        count_k, count_prev, count_next, count_union);
                    
                    % Move to next slice
                    k = k + 1;
                end
                
                % Safety check to prevent infinite loop
                if k > 100
                    fprintf('Safety break: too many iterations\n');
                    break;
                end
            end
            
            % Update slice indices to be consecutive
            for i = 1:length(obj.slices)
                obj.slices{i}.k = i - 1;  % 0-based indexing
            end
            
            obj.N = length(obj.slices) - 1;
            
            % Print final results
            final_count = length(obj.slices);
            reduction_percent = (removed_count / original_count) * 100;
            
            fprintf('\nSimplification Results:\n');
            fprintf('  Original slices: %d\n', original_count);
            fprintf('  Remaining slices: %d\n', final_count);
            fprintf('  Removed slices: %d\n', removed_count);
            fprintf('  Reduction: %.1f%%\n', reduction_percent);
            
            % Print remaining slice information
            fprintf('\nRemaining slice heights:\n');
            for i = 1:length(obj.slices)
                slice = obj.slices{i};
                traversable_count = sum(slice.cT(:) < obj.costParams.c_B & ~isnan(slice.cT(:)));
                fprintf('  Slice %d: z=%.2fm (%d traversable grids)\n', ...
                    i-1, slice.z, traversable_count);
            end
        end
        
        function visualizeSlices(obj)
            % Visualize all tomogram slices
            figure('Name', 'Tomogram 3D Analysis', 'Position', [100 100 1200 800]);
            
            % Define colors for traversability visualization
            traversable_color = [0.2 0.6 1.0];    % Blue for traversable
            blocked_color = [1.0 0.2 0.2];        % Red for blocked
            
            % Create 3D visualization
            hold on;
            
            % Convert grid coordinates to world coordinates
            [X, Y] = meshgrid(linspace(obj.minX, obj.maxX, obj.gridSize(2)), ...
                             linspace(obj.minY, obj.maxY, obj.gridSize(1)));
            
            % Process each slice
            for k = 1:length(obj.slices)
                slice = obj.slices{k};
                Z = ones(size(X)) * slice.z;
                
                % Only process valid ground points
                ground_valid = ~isnan(slice.eG);
                
                if any(ground_valid(:))
                    % Calculate traversability with strict threshold
                    traversable = ground_valid & (slice.cT < obj.costParams.c_B);
                    blocked = ground_valid & (~traversable);
                    
                    % Plot blocked areas first so traversable (blue)
                    % surfaces remain visible on top
                    if any(blocked(:))
                        X_masked = X;
                        Y_masked = Y;
                        Z_masked = Z;
                        X_masked(~blocked) = NaN;
                        Y_masked(~blocked) = NaN;
                        Z_masked(~blocked) = NaN;
                        
                        surf(X_masked, Y_masked, Z_masked, ...
                             'FaceColor', blocked_color, ...
                             'EdgeColor', 'none', ...
                             'FaceAlpha', 0.6);
                    end
                    
                    % Plot traversable areas afterwards so they're not
                    % hidden underneath the blocked surfaces
                    if any(traversable(:))
                        X_masked = X;
                        Y_masked = Y;
                        Z_masked = Z;
                        X_masked(~traversable) = NaN;
                        Y_masked(~traversable) = NaN;
                        Z_masked(~traversable) = NaN;
                        
                        surf(X_masked, Y_masked, Z_masked, ...
                             'FaceColor', traversable_color, ...
                             'EdgeColor', 'none', ...
                             'FaceAlpha', 0.9);
                    end
                end
            end
            
            % Adjust view
            grid on;
            title('3D Layer Analysis (Interactive View)');
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            view(45, 30);
            axis equal;
            
            % Enable rotation
            rotate3d on;
            
            % Add legend
            h1 = surf(nan(2), nan(2), nan(2), 'FaceColor', traversable_color, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            h2 = surf(nan(2), nan(2), nan(2), 'FaceColor', blocked_color, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
            legend([h1, h2], {'Traversable', 'Blocked'}, 'Location', 'eastoutside');
        end
        
        function path = planPath(obj, startPos, endPos)
            % Plan a path from startPos to endPos using modified A* for tomogram slices
            % Following PCT paper "Path Planning through Slices" section
            
            % Convert world coordinates to grid indices
            startX = startPos(1); startY = startPos(2); startZ = startPos(3);
            endX = endPos(1); endY = endPos(2); endZ = endPos(3);
            
            % Find which slice contains the start and end positions
            startSliceIdx = obj.findSliceContaining(startZ);
            endSliceIdx = obj.findSliceContaining(endZ);
            
            fprintf('=== PATH PLANNING DIAGNOSTICS ===\n');
            fprintf('Start position: [%.2f, %.2f, %.2f]\n', startX, startY, startZ);
            fprintf('End position: [%.2f, %.2f, %.2f]\n', endX, endY, endZ);
            fprintf('Available slices: %d\n', length(obj.slices));
            for i = 1:length(obj.slices)
                slice = obj.slices{i};
                fprintf('  Slice %d: z=%.2fm (range: %.2f to %.2f)\n', ...
                    i, slice.z, slice.z - obj.ds/2, slice.z + obj.ds/2);
            end
            fprintf('Start slice index: %d\n', startSliceIdx);
            fprintf('End slice index: %d\n', endSliceIdx);
            
            if startSliceIdx == 0 || endSliceIdx == 0
                fprintf('Start or end position not in any slice!\n');
                path = [];
                return;
            end
            
            startI = max(1, min(obj.gridSize(1), ceil((startY - obj.minY) / obj.rg)));
            startJ = max(1, min(obj.gridSize(2), ceil((startX - obj.minX) / obj.rg)));
            
            endI = max(1, min(obj.gridSize(1), ceil((endY - obj.minY) / obj.rg)));
            endJ = max(1, min(obj.gridSize(2), ceil((endX - obj.minX) / obj.rg)));
            
            fprintf('Start grid position: [%d, %d, %d]\n', startI, startJ, startSliceIdx);
            fprintf('End grid position: [%d, %d, %d]\n', endI, endJ, endSliceIdx);
            
            % Detailed traversability analysis for start position
            fprintf('\n=== START POSITION ANALYSIS ===\n');
            obj.analyzeGridPosition(startI, startJ, startSliceIdx, 'START');
            
            % Detailed traversability analysis for end position
            fprintf('\n=== END POSITION ANALYSIS ===\n');
            obj.analyzeGridPosition(endI, endJ, endSliceIdx, 'END');
            
            % Check if start and end positions are traversable
            startTraversable = obj.isTraversable(startI, startJ, startSliceIdx);
            endTraversable = obj.isTraversable(endI, endJ, endSliceIdx);
            
            if ~startTraversable || ~endTraversable
                fprintf('\n=== TRAVERSABILITY SUMMARY ===\n');
                fprintf('Start traversable: %d\n', startTraversable);
                fprintf('End traversable: %d\n', endTraversable);
                
                % Try to find nearest traversable positions
                fprintf('\n=== SEARCHING FOR NEARBY TRAVERSABLE POSITIONS ===\n');
                if ~startTraversable
                    obj.findNearestTraversable(startI, startJ, startSliceIdx, 'START');
                end
                if ~endTraversable
                    obj.findNearestTraversable(endI, endJ, endSliceIdx, 'END');
                end
                
                path = [];
                return;
            end
            
            % Initialize A* data structures with 3D nodes
            % Each node is [i, j, slice_idx, gScore, fScore]
            openSet = [startI, startJ, startSliceIdx, 0, ...
                      obj.heuristic([startI, startJ, startSliceIdx], [endI, endJ, endSliceIdx])];
            
            cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');
            gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            fScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            % Initialize scores
            startKey = obj.nodeToKey([startI, startJ, startSliceIdx]);
            gScore(startKey) = 0;
            fScore(startKey) = obj.heuristic([startI, startJ, startSliceIdx], [endI, endJ, endSliceIdx]);
            
            % A* main loop
            while ~isempty(openSet)
                % Extract the node with the lowest fScore
                [~, idxMin] = min(openSet(:, 5));
                current = openSet(idxMin, :);
                openSet(idxMin, :) = [];
                currentNode = current(1:3);
                currentKey = obj.nodeToKey(currentNode);
                
                % Check if goal reached
                if all(currentNode == [endI, endJ, endSliceIdx])
                    path = obj.reconstructPath(cameFrom, currentNode);
                    fprintf('Path found with %d steps!\n', size(path,1));
                    return;
                end
                
                % Get neighbors following the paper's approach
                neighbors = obj.getNeighborsWithGateways(currentNode);
                for n = 1:size(neighbors,1)
                    neighborNode = neighbors(n,:);
                    neighborKey = obj.nodeToKey(neighborNode);
                    
                    % Calculate movement cost using paper's method
                    moveCost = obj.calculateNodeCost(currentNode, neighborNode);
                    if isinf(moveCost)
                        continue;  % Skip impassable transitions
                    end
                    
                    tentative_gScore = gScore(currentKey) + moveCost;
                    
                    if ~isKey(gScore, neighborKey) || tentative_gScore < gScore(neighborKey)
                        % This path is better than any previous one
                        cameFrom(neighborKey) = currentNode;
                        gScore(neighborKey) = tentative_gScore;
                        f = tentative_gScore + obj.heuristic(neighborNode, [endI, endJ, endSliceIdx]);
                        fScore(neighborKey) = f;
                        openSet = [openSet; neighborNode, tentative_gScore, f];
                    end
                end
            end
            
            fprintf('No path found!\n');
            path = [];
        end
        
        function sliceIdx = findSliceContaining(obj, z)
            % Find which slice contains the given z coordinate
            sliceIdx = 0;
            for k = 1:length(obj.slices)
                slice = obj.slices{k};
                if z >= slice.z - obj.ds/2 && z <= slice.z + obj.ds/2
                    sliceIdx = k;  % Return 1-based slice index for MATLAB
                    return;
                end
            end
            % If no slice found, return 0
        end
        
        function neighbors = getNeighborsWithGateways(obj, node)
            % Get neighbors following the paper's gateway approach
            i = node(1); j = node(2); k = node(3);
            neighbors = [];
            
            % 8-connected neighbors in same slice
            for di = -1:1
                for dj = -1:1
                    if di == 0 && dj == 0
                        continue;
                    end
                    ni = i + di;
                    nj = j + dj;
                    
                    if ni >= 1 && ni <= obj.gridSize(1) && ...
                       nj >= 1 && nj <= obj.gridSize(2) && ...
                       obj.isTraversable(ni, nj, k)
                        neighbors = [neighbors; ni, nj, k];
                    end
                end
            end
            
            % Check gateway connections to adjacent slices
            % According to paper: check grids at same planimetric position (i,j)
            current_slice = obj.slices{k};
            current_ground = current_slice.eG(i,j);
            current_cost = current_slice.cT(i,j);
            
            % Check upper slice (k+1)
            if k < length(obj.slices)
                upper_slice = obj.slices{k+1};
                if ~isnan(upper_slice.eG(i,j)) && ~isnan(upper_slice.cT(i,j))
                    upper_ground = upper_slice.eG(i,j);
                    upper_cost = upper_slice.cT(i,j);
                    
                    % Gateway condition from paper: same ground elevation
                    if abs(current_ground - upper_ground) < 0.01  % Same ground elevation
                        % This is considered the same node in 3D space
                        % Choose the slice with lower cost (paper's c^N = min(c^T_k, c^T_{k+1}))
                        if upper_cost <= current_cost && obj.isTraversable(i, j, k+1)
                            neighbors = [neighbors; i, j, k+1];
                        end
                    else
                        % Different ground elevations - check if it's a valid transition
                        if upper_cost <= current_cost && obj.isTraversable(i, j, k+1)
                            neighbors = [neighbors; i, j, k+1];
                        end
                    end
                end
            end
            
            % Check lower slice (k-1)
            if k > 1
                lower_slice = obj.slices{k-1};
                if ~isnan(lower_slice.eG(i,j)) && ~isnan(lower_slice.cT(i,j))
                    lower_ground = lower_slice.eG(i,j);
                    lower_cost = lower_slice.cT(i,j);
                    
                    % Gateway condition from paper
                    if abs(current_ground - lower_ground) < 0.01  % Same ground elevation
                        % Same node in 3D space
                        if lower_cost <= current_cost && obj.isTraversable(i, j, k-1)
                            neighbors = [neighbors; i, j, k-1];
                        end
                    else
                        % Different ground elevations
                        if lower_cost <= current_cost && obj.isTraversable(i, j, k-1)
                            neighbors = [neighbors; i, j, k-1];
                        end
                    end
                end
            end
        end
        
        function cost = calculateNodeCost(obj, node1, node2)
            % Calculate movement cost between nodes following the paper's method
            i1 = node1(1); j1 = node1(2); k1 = node1(3);
            i2 = node2(1); j2 = node2(2); k2 = node2(3);
            
            % Check if nodes are traversable
            if ~obj.isTraversable(i1, j1, k1) || ~obj.isTraversable(i2, j2, k2)
                cost = inf;
                return;
            end
            
            slice1 = obj.slices{k1};
            slice2 = obj.slices{k2};
            
            % Get node costs following paper's c^N formula
            if k1 == k2
                % Same slice - use regular traversability cost
                node_cost = slice1.cT(i2, j2);
            else
                % Different slices - check if they represent the same 3D node
                ground1 = slice1.eG(i1, j1);
                ground2 = slice2.eG(i2, j2);
                
                if i1 == i2 && j1 == j2 && abs(ground1 - ground2) < 0.01
                    % Same 3D node: c^N = min(c^T_{i,j,k}, c^T_{i,j,k+1})
                    node_cost = min(slice1.cT(i1, j1), slice2.cT(i2, j2));
                else
                    % Different nodes - use target node cost
                    node_cost = slice2.cT(i2, j2);
                end
            end
            
            % Check if connection breaks (c^N = c^B)
            if node_cost > obj.costParams.c_B
                cost = inf;
                return;
            end
            
            % Calculate Euclidean distance
            if k1 == k2
                % Same slice
                dist = sqrt((i2-i1)^2 + (j2-j1)^2);
            else
                % Different slices - include vertical component
                z1 = slice1.z;
                z2 = slice2.z;
                dist = sqrt((i2-i1)^2 + (j2-j1)^2 + ((z2-z1)/obj.ds)^2);
            end
            
            % Final cost: node cost + Euclidean distance
            cost = node_cost + dist;
        end
        
        function h = heuristic(obj, node1, node2)
            % Manhattan distance heuristic with height difference
            xyDist = abs(node1(1) - node2(1)) + abs(node1(2) - node2(2));
            zDist = abs(node1(3) - node2(3)) * 2;  % Weight vertical movement more
            h = xyDist + zDist;
        end
        
        function key = nodeToKey(~, node)
            % Convert node coordinates to string key
            key = sprintf('%d_%d_%d', node(1), node(2), node(3));
        end
        
        function path = reconstructPath(obj, cameFrom, current)
            % Reconstruct path from cameFrom map
            path = current;
            currentKey = obj.nodeToKey(current);
            
            while isKey(cameFrom, currentKey)
                current = cameFrom(currentKey);
                currentKey = obj.nodeToKey(current);
                path = [current; path];
            end
        end
        
        function valid = isTraversable(obj, i, j, k)
            % Enhanced traversability check considering clearance and ground conditions
            if k < 1 || k > length(obj.slices) || ...
               i < 1 || i > obj.gridSize(1) || ...
               j < 1 || j > obj.gridSize(2)
                valid = false;
                return;
            end
            
            slice = obj.slices{k};
            
            % Check basic traversability cost
            if isnan(slice.cT(i,j)) || slice.cT(i,j) > obj.costParams.c_B
                valid = false;
                return;
            end
            
            % Check clearance
            clearance = slice.eC(i,j) - slice.eG(i,j);
            if clearance < obj.robotParams.d_min
                valid = false;
                return;
            end
            
            % Check ground slope if available
            if isfield(slice, 'groundSlope')
                if slice.groundSlope(i,j) > obj.robotParams.theta_max
                    valid = false;
                    return;
                end
            end
            
            % Additional check for support
            support = slice.z - slice.eG(i,j);
            if support > 3.5*obj.ds  % 0.7m
                valid = false;
                return;
            end
            
            valid = true;
        end
        
        function visualizePathAndSlices(obj, path)
            % Visualize both the tomogram slices and the planned path
            obj.visualizeSlices();
            hold on;
            
            if ~isempty(path)
                % Convert path indices to world coordinates
                pathX = obj.minX + (path(:,2) - 1) * obj.rg;
                pathY = obj.minY + (path(:,1) - 1) * obj.rg;
                pathZ = zeros(size(path,1), 1);
                for i = 1:size(path,1)
                    pathZ(i) = obj.slices{path(i,3)}.z;
                end
                
                % Plot path
                plot3(pathX, pathY, pathZ, 'g.-', 'LineWidth', 2, 'MarkerSize', 10);
                legend('Traversable', 'Blocked', 'Planned Path');
            end
            
            hold off;
        end
        
        function optimizedPath = optimizePCTTrajectory(obj, path, robotParams)
            % PCT Trajectory Optimization following paper Equation (11)
            % M-piece 3D polynomials with proper constraints
            
            if size(path, 1) < 3
                optimizedPath = path;
                return;
            end
            
            fprintf('\n=== PCT Trajectory Optimization (Paper Equation 11) ===\n');
            
            % Convert grid path to world coordinates (this is our reference trajectory)
            worldPath = obj.convertPathToWorld(path);
            
            % Determine number of pieces (M) based on path complexity
            M = obj.determineOptimalPieces(worldPath);
            fprintf('Using M=%d pieces for trajectory optimization\n', M);
            
            % Time parameterization
            totalDist = sum(sqrt(sum(diff(worldPath).^2, 2)));
            T_total = totalDist / 0.3;  % 0.3 m/s average speed
            T_pieces = linspace(0, T_total, M+1);  % Time breakpoints
            
            % Create reference trajectory functions Zref(q(t))
            [Zref_func, t_ref] = obj.createReferenceTrajectory(worldPath, T_total);
            
            % Initialize optimization variables
            % Each piece has 6 coefficients (σ0, σ1, ..., σ5) for each dimension (x,y,z)
            numVars = M * 6 * 3;  % M pieces × 6 coefficients × 3 dimensions
            
            % Create constraint matrices and vectors
            [Aeq, beq, A, b, lb, ub] = obj.buildConstraintMatrices(worldPath, M, T_pieces, robotParams);
            
            % Define cost function (Equation 11a)
            costFun = @(vars) obj.evaluatePCTCost(vars, M, T_pieces, Zref_func, t_ref, robotParams);
            
            % Initial guess: simple interpolation between waypoints
            x0 = obj.generateInitialGuess(worldPath, M, T_pieces);
            
            % Solve optimization problem
            fprintf('Solving trajectory optimization with %d variables...\n', numVars);
            options = optimoptions('fmincon', ...
                'Display', 'iter-detailed', ...
                'Algorithm', 'interior-point', ...
                'MaxIterations', 1000, ...
                'MaxFunctionEvaluations', 5000, ...
                'ConstraintTolerance', 1e-6, ...
                'OptimalityTolerance', 1e-6);
            
            try
                [vars_opt, fval, exitflag] = fmincon(costFun, x0, A, b, Aeq, beq, lb, ub, [], options);
                
                if exitflag > 0
                    fprintf('Optimization successful! Cost: %.6f\n', fval);
                    
                    % Generate final trajectory with high resolution for smoothness
                    numHighResPoints = max(200, size(worldPath,1) * 5);  % At least 200 points or 5x original
                    optimizedPath = obj.generateTrajectoryFromCoeffs(vars_opt, M, T_pieces, numHighResPoints);
                    
                    % Verify constraints
                    obj.verifyTrajectoryConstraints(optimizedPath, robotParams);
                    
                else
                    fprintf('Optimization failed (exitflag=%d), using reference path\n', exitflag);
                    optimizedPath = worldPath;
                end
                
            catch ME
                fprintf('Optimization error: %s\n', ME.message);
                fprintf('Using reference path as fallback\n');
                optimizedPath = worldPath;
            end
            
            % Analyze trajectory quality
            obj.analyzePCTTrajectory(worldPath, optimizedPath, M, T_pieces);
        end
        
        function M = determineOptimalPieces(obj, worldPath)
            % Determine optimal number of pieces based on path complexity
            
            % Analyze path curvature and height changes
            pathLen = size(worldPath, 1);
            
            % Calculate curvature at each point
            curvatures = zeros(pathLen-2, 1);
            for i = 2:pathLen-1
                v1 = worldPath(i,:) - worldPath(i-1,:);
                v2 = worldPath(i+1,:) - worldPath(i,:);
                
                if norm(v1) > 0 && norm(v2) > 0
                    cosAngle = dot(v1, v2) / (norm(v1) * norm(v2));
                    curvatures(i-1) = acos(max(-1, min(1, cosAngle)));
                end
            end
            
            % Calculate height changes
            heightChanges = abs(diff(worldPath(:,3)));
            
            % Determine number of pieces based on complexity
            highCurvature = sum(curvatures > 0.5);  % 30 degrees
            significantHeightChange = sum(heightChanges > 0.1);  % 10cm
            
            M = max(3, min(8, ceil((highCurvature + significantHeightChange) / 3)));
            fprintf('Path analysis: %d high curvature points, %d significant height changes\n', ...
                highCurvature, significantHeightChange);
        end
        
        function [Zref_func, t_ref] = createReferenceTrajectory(obj, worldPath, T_total)
            % Create reference trajectory function Zref(q(t)) from A* path
            
            % Time parameterization for reference path
            distances = [0; cumsum(sqrt(sum(diff(worldPath).^2, 2)))];
            t_ref = distances / distances(end) * T_total;
            
            % Create interpolation functions for reference trajectory
            Zref_func = struct();
            Zref_func.x = @(t) interp1(t_ref, worldPath(:,1), t, 'pchip', 'extrap');
            Zref_func.y = @(t) interp1(t_ref, worldPath(:,2), t, 'pchip', 'extrap');
            Zref_func.z = @(t) interp1(t_ref, worldPath(:,3), t, 'pchip', 'extrap');
            
            fprintf('Created reference trajectory with %d waypoints over %.2fs\n', ...
                size(worldPath,1), T_total);
        end
        
        function [Aeq, beq, A, b, lb, ub] = buildConstraintMatrices(obj, worldPath, M, T_pieces, robotParams)
            % Build constraint matrices for PCT optimization (Equations 11b-11f)
            
            numVars = M * 6 * 3;  % M pieces × 6 coefficients × 3 dimensions
            
            % Boundary conditions (11b): q1(0) = q̄0, qM(T) = q̄f
            % Continuity constraints (11c): qi[3](Ti) = qi+1[3](0) for i=1,...,M-1
            
            numEqConstraints = 6 + 3*(M-1)*4;  % Start/end (6) + continuity (3*(M-1)*4)
            Aeq = zeros(numEqConstraints, numVars);
            beq = zeros(numEqConstraints, 1);
            
            % Helper function for polynomial basis
            beta = @(t) [1, t, t^2, t^3, t^4, t^5];
            dbeta = @(t) [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4];
            ddbeta = @(t) [0, 0, 2, 6*t, 12*t^2, 20*t^3];
            dddbeta = @(t) [0, 0, 0, 6, 24*t, 60*t^2];
            
            rowIdx = 1;
            
            % Start position constraints: q1(0) = [x0, y0, z0]
            for dim = 1:3
                coeffIdx = (dim-1)*M*6 + (1:6);  % First piece, current dimension
                Aeq(rowIdx, coeffIdx) = beta(0);
                beq(rowIdx) = worldPath(1, dim);
                rowIdx = rowIdx + 1;
            end
            
            % End position constraints: qM(T_M) = [xf, yf, zf]
            T_M = T_pieces(end) - T_pieces(end-1);
            for dim = 1:3
                coeffIdx = (dim-1)*M*6 + (M-1)*6 + (1:6);  % Last piece, current dimension
                Aeq(rowIdx, coeffIdx) = beta(T_M);
                beq(rowIdx) = worldPath(end, dim);
                rowIdx = rowIdx + 1;
            end
            
            % Continuity constraints (11c) for each piece junction
            for i = 1:M-1
                T_i = T_pieces(i+1) - T_pieces(i);
                
                for dim = 1:3
                    % Position continuity: qi(Ti) = qi+1(0)
                    coeffIdx_i = (dim-1)*M*6 + (i-1)*6 + (1:6);
                    coeffIdx_i1 = (dim-1)*M*6 + i*6 + (1:6);
                    Aeq(rowIdx, coeffIdx_i) = beta(T_i);
                    Aeq(rowIdx, coeffIdx_i1) = -beta(0);
                    beq(rowIdx) = 0;
                    rowIdx = rowIdx + 1;
                    
                    % Velocity continuity: qi'(Ti) = qi+1'(0)
                    Aeq(rowIdx, coeffIdx_i) = dbeta(T_i);
                    Aeq(rowIdx, coeffIdx_i1) = -dbeta(0);
                    beq(rowIdx) = 0;
                    rowIdx = rowIdx + 1;
                    
                    % Acceleration continuity: qi''(Ti) = qi+1''(0)
                    Aeq(rowIdx, coeffIdx_i) = ddbeta(T_i);
                    Aeq(rowIdx, coeffIdx_i1) = -ddbeta(0);
                    beq(rowIdx) = 0;
                    rowIdx = rowIdx + 1;
                    
                    % Jerk continuity: qi'''(Ti) = qi+1'''(0)
                    Aeq(rowIdx, coeffIdx_i) = dddbeta(T_i);
                    Aeq(rowIdx, coeffIdx_i1) = -dddbeta(0);
                    beq(rowIdx) = 0;
                    rowIdx = rowIdx + 1;
                end
            end
            
            % Inequality constraints will be handled in nonlinear constraints
            A = [];
            b = [];
            
            % Variable bounds
            lb = -inf(numVars, 1);
            ub = inf(numVars, 1);
            
            fprintf('Built constraint matrices: %d equality constraints, %d variables\n', ...
                size(Aeq,1), numVars);
        end
        
        function cost = evaluatePCTCost(obj, vars, M, T_pieces, Zref_func, t_ref, robotParams)
            % Evaluate cost function from Equation 11a
            
            cost = 0;
            w2 = 10.0;    % Weight for reference tracking
            wT = 0.1;     % Weight for time penalty
            
            % Polynomial basis functions
            beta = @(t) [1, t, t^2, t^3, t^4, t^5];
            dddbeta = @(t) [0, 0, 0, 6, 24*t, 60*t^2];
            
            % Evaluate cost for each piece
            for i = 1:M
                T_i = T_pieces(i+1) - T_pieces(i);
                t_start = T_pieces(i);
                
                % Extract coefficients for this piece
                x_coeffs = vars((0*M + i-1)*6 + (1:6));
                y_coeffs = vars((1*M + i-1)*6 + (1:6));
                z_coeffs = vars((2*M + i-1)*6 + (1:6));
                
                % Jerk cost (Jc): ∫ ||q'''(t)||² dt
                jerk_cost_x = obj.integrateJerkSquared(x_coeffs, T_i);
                jerk_cost_y = obj.integrateJerkSquared(y_coeffs, T_i);
                jerk_cost_z = obj.integrateJerkSquared(z_coeffs, T_i);
                Jc = jerk_cost_x + jerk_cost_y + jerk_cost_z;
                
                % Reference tracking cost: ∫ ||qz(t) - Zref(q(t))||² dt
                ref_cost = 0;
                num_samples = 10;
                for j = 1:num_samples
                    t_local = (j-1) * T_i / (num_samples-1);
                    t_global = t_start + t_local;
                    
                    % Evaluate polynomial at this time
                    q_t = [
                        beta(t_local) * x_coeffs;
                        beta(t_local) * y_coeffs;
                        beta(t_local) * z_coeffs
                    ];
                    
                    % Get reference height at this position
                    z_ref = Zref_func.z(t_global);
                    
                    % Reference tracking error
                    ref_error = (q_t(3) - z_ref)^2;
                    ref_cost = ref_cost + ref_error * T_i / num_samples;
                end
                
                % Combine costs for this piece
                piece_cost = Jc + w2 * ref_cost;
                cost = cost + piece_cost;
            end
            
            % Time penalty
            total_time = T_pieces(end);
            cost = cost + wT * total_time;
        end
        
        function jerk_integral = integrateJerkSquared(obj, coeffs, T)
            % Analytically integrate ||q'''(t)||² from 0 to T
            % For quintic polynomial q(t) = σ0 + σ1*t + ... + σ5*t^5
            % q'''(t) = 6*σ3 + 24*σ4*t + 60*σ5*t^2
            
            s3 = coeffs(4);
            s4 = coeffs(5);
            s5 = coeffs(6);
            
            % ∫[0,T] [6*σ3 + 24*σ4*t + 60*σ5*t^2]² dt
            term1 = (6*s3)^2 * T;
            term2 = 2*(6*s3)*(24*s4) * T^2 / 2;
            term3 = (2*(6*s3)*(60*s5) + (24*s4)^2) * T^3 / 3;
            term4 = 2*(24*s4)*(60*s5) * T^4 / 4;
            term5 = (60*s5)^2 * T^5 / 5;
            
            jerk_integral = term1 + term2 + term3 + term4 + term5;
        end
        
        function x0 = generateInitialGuess(obj, worldPath, M, T_pieces)
            % Generate initial guess using simple interpolation
            
            numVars = M * 6 * 3;
            x0 = zeros(numVars, 1);
            
            % For each dimension
            for dim = 1:3
                % Create time-parameterized path
                path_times = linspace(0, T_pieces(end), size(worldPath,1));
                path_values = worldPath(:, dim);
                
                % For each piece
                for i = 1:M
                    T_i = T_pieces(i+1) - T_pieces(i);
                    t_start = T_pieces(i);
                    t_end = T_pieces(i+1);
                    
                    % Sample points in this piece
                    t_samples = linspace(t_start, t_end, 6);
                    y_samples = interp1(path_times, path_values, t_samples, 'pchip');
                    
                    % Fit polynomial to these points (simple least squares)
                    A_fit = zeros(6, 6);
                    for j = 1:6
                        t_local = t_samples(j) - t_start;
                        A_fit(j, :) = [1, t_local, t_local^2, t_local^3, t_local^4, t_local^5];
                    end
                    
                    coeffs = A_fit \ y_samples';
                    
                    % Store coefficients
                    coeffIdx = (dim-1)*M*6 + (i-1)*6 + (1:6);
                    x0(coeffIdx) = coeffs;
                end
            end
            
            fprintf('Generated initial guess with %d variables\n', numVars);
        end
        
        function trajectory = generateTrajectoryFromCoeffs(obj, vars, M, T_pieces, numPoints)
            % Generate trajectory from optimized coefficients with high resolution
            
            trajectory = zeros(numPoints, 3);
            
            % Create high-resolution time vector
            t_trajectory = linspace(0, T_pieces(end), numPoints);
            
            beta = @(t) [1, t, t^2, t^3, t^4, t^5];
            
            fprintf('Generating %d-point trajectory from %d polynomial pieces...\n', numPoints, M);
            
            for k = 1:numPoints
                t = t_trajectory(k);
                
                % Find which piece this time belongs to
                piece_idx = find(t >= T_pieces(1:end-1) & t < T_pieces(2:end), 1);
                if isempty(piece_idx)
                    if t <= T_pieces(1)
                        piece_idx = 1;
                    else
                        piece_idx = M;  % Handle end boundary case
                    end
                end
                
                % Local time within the piece (normalized to [0, T_piece])
                t_local = t - T_pieces(piece_idx);
                
                % Extract coefficients for this piece and evaluate polynomial
                for dim = 1:3
                    coeffIdx = (dim-1)*M*6 + (piece_idx-1)*6 + (1:6);
                    coeffs = vars(coeffIdx);
                    trajectory(k, dim) = beta(t_local) * coeffs;
                end
            end
            
            % Report trajectory smoothness
            obj.reportTrajectoryQuality(trajectory, T_pieces(end));
        end
        
        function reportTrajectoryQuality(obj, trajectory, totalTime)
            % Report trajectory quality metrics
            
            dt = totalTime / (size(trajectory,1) - 1);
            
            fprintf('Trajectory Quality Report:\n');
            fprintf('  Resolution: %d points over %.2fs (dt=%.3fs)\n', ...
                size(trajectory,1), totalTime, dt);
            
            % Calculate velocity, acceleration, and jerk
            vel = diff(trajectory) / dt;
            acc = diff(vel) / dt;
            jerk = diff(acc) / dt;
            
            % Report maximum values
            max_vel = max(sqrt(sum(vel.^2, 2)));
            max_acc = max(sqrt(sum(acc.^2, 2)));
            max_jerk = max(sqrt(sum(jerk.^2, 2)));
            
            fprintf('  Max velocity: %.3f m/s\n', max_vel);
            fprintf('  Max acceleration: %.3f m/s²\n', max_acc);
            fprintf('  Max jerk: %.3f m/s³\n', max_jerk);
            
            % Check for smoothness (no sudden jumps)
            position_jumps = max(sqrt(sum(diff(trajectory).^2, 2)));
            if position_jumps > 0.1  % 10cm jump threshold
                fprintf('  WARNING: Large position jumps detected (%.3fm)\n', position_jumps);
            else
                fprintf('  Position continuity: GOOD (max jump %.3fm)\n', position_jumps);
            end
        end
        
        function verifyTrajectoryConstraints(obj, trajectory, robotParams)
            % Verify that trajectory satisfies height constraints (11f)
            
            violations = 0;
            for i = 1:size(trajectory, 1)
                [Hg, Hc] = obj.queryHeightsAtPosition(trajectory(i,1), trajectory(i,2));
                
                if ~isnan(Hg) && ~isnan(Hc)
                    if trajectory(i,3) < Hg || trajectory(i,3) > Hc
                        violations = violations + 1;
                    end
                end
            end
            
            fprintf('Trajectory verification: %d/%d points violate height constraints\n', ...
                violations, size(trajectory,1));
        end
        
        function analyzePCTTrajectory(obj, originalPath, optimizedPath, M, T_pieces)
            % Analyze trajectory quality and smoothness
            
            fprintf('\n--- PCT Trajectory Analysis ---\n');
            fprintf('Number of pieces: %d\n', M);
            fprintf('Total time: %.2fs\n', T_pieces(end));
            
            % Path length comparison
            orig_length = sum(sqrt(sum(diff(originalPath).^2, 2)));
            opt_length = sum(sqrt(sum(diff(optimizedPath).^2, 2)));
            fprintf('Path length: %.2fm → %.2fm (%.1f%% change)\n', ...
                orig_length, opt_length, (opt_length/orig_length-1)*100);
            
            % Height difference
            height_diff = optimizedPath(end,3) - optimizedPath(1,3);
            fprintf('Height gain: %.2fm\n', height_diff);
            
            % Smoothness analysis
            dt = T_pieces(end) / (size(optimizedPath,1)-1);
            for dim = 1:3
                dimName = {'X', 'Y', 'Z'};
                
                % Calculate derivatives
                vel = diff(optimizedPath(:,dim)) / dt;
                acc = diff(vel) / dt;
                jerk = diff(acc) / dt;
                
                fprintf('%s: Max vel=%.2f, Max acc=%.2f, RMS jerk=%.2f\n', ...
                    dimName{dim}, max(abs(vel)), max(abs(acc)), rms(jerk));
            end
        end
        
        function [groundHeight, ceilingHeight] = queryHeightsAtPosition(obj, x, y)
            % Query Hg and Hc at position (x,y) from tomogram
            groundHeight = NaN;
            ceilingHeight = NaN;
            
            % Check bounds
            if x < obj.minX || x > obj.maxX || y < obj.minY || y > obj.maxY
                return;
            end
            
            % Convert to grid coordinates
            i = max(1, min(obj.gridSize(1), ceil((y - obj.minY) / obj.rg)));
            j = max(1, min(obj.gridSize(2), ceil((x - obj.minX) / obj.rg)));
            
            % Find the best slice for this position (use middle slice as representative)
            if ~isempty(obj.slices)
                midSliceIdx = ceil(length(obj.slices) / 2);
                slice = obj.slices{midSliceIdx};
                
                if i >= 1 && i <= obj.gridSize(1) && j >= 1 && j <= obj.gridSize(2)
                    if ~isnan(slice.eG(i, j)) && ~isnan(slice.eC(i, j))
                        groundHeight = slice.eG(i, j);
                        ceilingHeight = slice.eC(i, j);
                    end
                end
            end
        end
        
        function visualizeOptimizedTrajectory(obj, originalPath, optimizedPath)
            % Visualize original and optimized trajectories
            figure('Name', 'Trajectory Optimization Results', 'Position', [100, 100, 1200, 800]);
            
            % 3D view
            subplot(2,2,[1,3]);
            obj.visualizeSlices();
            hold on;
            plot3(originalPath(:,1), originalPath(:,2), originalPath(:,3), 'r--', 'LineWidth', 1, 'DisplayName', 'Original Path');
            plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), 'g-', 'LineWidth', 2, 'DisplayName', 'Optimized Path');
            plot3(originalPath(1,1), originalPath(1,2), originalPath(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
            plot3(originalPath(end,1), originalPath(end,2), originalPath(end,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('3D View of Trajectories');
            legend('Location', 'best');
            view(45, 30);
            grid on;
            
            % XY view
            subplot(2,2,2);
            plot(originalPath(:,1), originalPath(:,2), 'r--', 'LineWidth', 1);
            hold on;
            plot(optimizedPath(:,1), optimizedPath(:,2), 'g-', 'LineWidth', 2);
            plot(originalPath(1,1), originalPath(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            plot(originalPath(end,1), originalPath(end,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            xlabel('X (m)'); ylabel('Y (m)');
            title('Top View');
            axis equal;
            grid on;
            
            % Height profile
            subplot(2,2,4);
            plot(1:size(originalPath,1), originalPath(:,3), 'r--', 'LineWidth', 1);
            hold on;
            plot(1:size(optimizedPath,1), optimizedPath(:,3), 'g-', 'LineWidth', 2);
            xlabel('Waypoint Index'); ylabel('Height (m)');
            title('Height Profile');
            grid on;
            
            % Add trajectory statistics
            origDist = sum(sqrt(sum(diff(originalPath).^2, 2)));
            optDist = sum(sqrt(sum(diff(optimizedPath).^2, 2)));
            fprintf('\nTrajectory Statistics:\n');
            fprintf('Original path length: %.2f m\n', origDist);
            fprintf('Optimized path length: %.2f m\n', optDist);
            fprintf('Length reduction: %.1f%%\n', (1 - optDist/origDist) * 100);
        end
        
        function plotCostTrends(obj)
            % Plot mean CI, CG, CInit across all slices
            n = length(obj.slices);
            meanCI   = nan(1,n);
            meanCG   = nan(1,n);
            meanInit = nan(1,n);
            for k = 1:n
                sl = obj.slices{k};
                meanCI(k)   = mean(sl.cI(~isnan(sl.cI)));     %#ok<NASGU>
                meanCG(k)   = mean(sl.cG(~isnan(sl.cG)));
                meanInit(k) = mean(sl.cInit(~isnan(sl.cInit)));
            end
            figure('Name','Cost Trends Across Slices');
            plot(0:n-1, meanCI,'-o','LineWidth',1.5); hold on;
            plot(0:n-1, meanCG,'-s','LineWidth',1.5);
            plot(0:n-1, meanInit,'-d','LineWidth',1.5);
            grid on;
            xlabel('Slice index (k)');
            ylabel('Mean cost');
            legend({'c_I','c_G','c_{Init}'},'Location','best');
            title('Cost Component Trends Across Slices');
        end
        
        function analyzeGridPosition(obj, i, j, sliceIdx, label)
            % Detailed analysis of why a grid position may not be traversable
            fprintf('Analyzing %s position [%d, %d, %d]:\n', label, i, j, sliceIdx);
            
            if sliceIdx < 1 || sliceIdx > length(obj.slices)
                fprintf('  ERROR: Invalid slice index %d (valid range: 1-%d)\n', sliceIdx, length(obj.slices));
                return;
            end
            
            if i < 1 || i > obj.gridSize(1) || j < 1 || j > obj.gridSize(2)
                fprintf('  ERROR: Grid position out of bounds [%d,%d] (valid range: [1,%d] x [1,%d])\n', ...
                    i, j, obj.gridSize(1), obj.gridSize(2));
                return;
            end
            
            slice = obj.slices{sliceIdx};
            fprintf('  Slice height: %.2fm\n', slice.z);
            
            % Check ground and ceiling data
            eG = slice.eG(i,j);
            eC = slice.eC(i,j);
            fprintf('  Ground height (eG): %.3fm\n', eG);
            fprintf('  Ceiling height (eC): %.3fm\n', eC);
            
            if isnan(eG) || isnan(eC)
                fprintf('  ISSUE: Missing ground/ceiling data (NaN values)\n');
                return;
            end
            
            % Check clearance
            clearance = eC - eG;
            fprintf('  Clearance (dI): %.3fm\n', clearance);
            fprintf('  Required minimum clearance: %.3fm\n', obj.robotParams.d_min);
            
            if clearance < obj.robotParams.d_min
                fprintf('  ISSUE: Insufficient clearance (%.3f < %.3f)\n', clearance, obj.robotParams.d_min);
            else
                fprintf('  Clearance: OK\n');
            end
            
            % Check traversability cost
            cT = slice.cT(i,j);
            fprintf('  Traversability cost (cT): %.2f\n', cT);
            fprintf('  Barrier threshold (c_B): %.2f\n', obj.costParams.c_B);
            
            if isnan(cT)
                fprintf('  ISSUE: Traversability cost is NaN\n');
            elseif cT >= obj.costParams.c_B
                fprintf('  ISSUE: Cost too high (%.2f >= %.2f)\n', cT, obj.costParams.c_B);
            else
                fprintf('  Cost: OK\n');
            end
            
            % Check support distance
            support = slice.z - eG;
            supportThreshold = 1.4 * obj.ds;
            fprintf('  Support distance: %.3fm\n', support);
            fprintf('  Support threshold: %.3fm\n', supportThreshold);
            
            if support > supportThreshold
                fprintf('  ISSUE: Support too far (%.3f > %.3f)\n', support, supportThreshold);
            else
                fprintf('  Support: OK\n');
            end
            
            % Final traversability check
            traversable = obj.isTraversable(i, j, sliceIdx);
            if traversable
                fprintf('  Final traversable: YES\n');
            else
                fprintf('  Final traversable: NO\n');
            end
        end
        
        function findNearestTraversable(obj, centerI, centerJ, sliceIdx, label)
            % Find nearest traversable position within a search radius
            fprintf('Searching for nearest traversable position to %s [%d,%d,%d]:\n', ...
                label, centerI, centerJ, sliceIdx);
            
            maxRadius = 5;  % Search within 5 grid cells
            found = false;
            
            for radius = 1:maxRadius
                fprintf('  Checking radius %d...\n', radius);
                for di = -radius:radius
                    for dj = -radius:radius
                        if abs(di) ~= radius && abs(dj) ~= radius
                            continue;  % Only check perimeter of current radius
                        end
                        
                        ni = centerI + di;
                        nj = centerJ + dj;
                        
                        if ni >= 1 && ni <= obj.gridSize(1) && ...
                           nj >= 1 && nj <= obj.gridSize(2)
                            if obj.isTraversable(ni, nj, sliceIdx)
                                fprintf('    Found traversable at [%d,%d,%d] (offset [%d,%d])\n', ...
                                    ni, nj, sliceIdx, di, dj);
                                found = true;
                                
                                % Convert back to world coordinates
                                worldX = obj.minX + (nj - 1) * obj.rg;
                                worldY = obj.minY + (ni - 1) * obj.rg;
                                worldZ = obj.slices{sliceIdx}.z;
                                fprintf('    World coordinates: [%.2f, %.2f, %.2f]\n', ...
                                    worldX, worldY, worldZ);
                                return;
                            end
                        end
                    end
                end
            end
            
            if ~found
                fprintf('    No traversable position found within radius %d\n', maxRadius);
                
                % Count total traversable cells in this slice
                slice = obj.slices{sliceIdx};
                totalTraversable = 0;
                for i = 1:obj.gridSize(1)
                    for j = 1:obj.gridSize(2)
                        if obj.isTraversable(i, j, sliceIdx)
                            totalTraversable = totalTraversable + 1;
                        end
                    end
                end
                fprintf('    Total traversable cells in slice %d: %d/%d\n', ...
                    sliceIdx, totalTraversable, obj.gridSize(1) * obj.gridSize(2));
            end
        end
        
        function analyzeStairScenario(obj, startPos, endPos)
            % Comprehensive analysis of the stair traversal scenario
            fprintf('\n=== STAIR SCENARIO ANALYSIS ===\n');
            
            % Convert positions to grid coordinates
            startI = max(1, min(obj.gridSize(1), ceil((startPos(2) - obj.minY) / obj.rg)));
            startJ = max(1, min(obj.gridSize(2), ceil((startPos(1) - obj.minX) / obj.rg)));
            endI = max(1, min(obj.gridSize(1), ceil((endPos(2) - obj.minY) / obj.rg)));
            endJ = max(1, min(obj.gridSize(2), ceil((endPos(1) - obj.minX) / obj.rg)));
            
            fprintf('Analyzing path from grid [%d,%d] to [%d,%d]\n', startI, startJ, endI, endJ);
            
            % Analyze traversability along the direct path
            fprintf('\n--- Traversability Analysis Along Y-axis ---\n');
            totalGrids = abs(endI - startI) + 1;
            traversableCount = 0;
            blockedCount = 0;
            
            for slice_idx = 1:length(obj.slices)
                slice = obj.slices{slice_idx};
                fprintf('\nSlice %d (z=%.2fm):\n', slice_idx, slice.z);
                
                traversableInSlice = 0;
                blockedInSlice = 0;
                
                % Check each grid along the path
                minI = min(startI, endI);
                maxI = max(startI, endI);
                
                for i = minI:maxI
                    j = startJ; % Same x-coordinate
                    
                    if i >= 1 && i <= obj.gridSize(1) && j >= 1 && j <= obj.gridSize(2)
                        isTraversable = obj.isTraversable(i, j, slice_idx);
                        cost = slice.cT(i, j);
                        
                        if isTraversable
                            traversableInSlice = traversableInSlice + 1;
                        else
                            blockedInSlice = blockedInSlice + 1;
                            
                            % Analyze why it's blocked
                            if isnan(cost)
                                reason = 'NaN cost';
                            elseif cost >= obj.costParams.c_B
                                reason = sprintf('High cost (%.1f)', cost);
                            else
                                reason = 'Other';
                            end
                            
                            fprintf('  Grid [%d,%d]: BLOCKED (%s)\n', i, j, reason);
                        end
                    end
                end
                
                fprintf('  Traversable: %d, Blocked: %d\n', traversableInSlice, blockedInSlice);
                traversableCount = traversableCount + traversableInSlice;
                blockedCount = blockedCount + blockedInSlice;
            end
            
            fprintf('\n--- Overall Path Analysis ---\n');
            fprintf('Total grids checked: %d\n', totalGrids * length(obj.slices));
            fprintf('Traversable grids: %d\n', traversableCount);
            fprintf('Blocked grids: %d\n', blockedCount);
            fprintf('Traversability ratio: %.2f%%\n', (traversableCount / (traversableCount + blockedCount)) * 100);
            
                         % Analyze slice connectivity
             fprintf('\n--- Slice Connectivity Analysis ---\n');
             for slice_idx = 1:min(2, length(obj.slices)-1)  % Only analyze first 2 connections to avoid too much output
                currentSlice = obj.slices{slice_idx};
                nextSlice = obj.slices{slice_idx+1};
                
                % Check connectivity at the same grid position
                i = startI; j = startJ;
                
                currentTraversable = obj.isTraversable(i, j, slice_idx);
                nextTraversable = obj.isTraversable(i, j, slice_idx+1);
                
                                 currentStatus = 'BLOCKED';
                 if currentTraversable
                     currentStatus = 'OK';
                 end
                 nextStatus = 'BLOCKED';
                 if nextTraversable
                     nextStatus = 'OK';
                 end
                 
                                  fprintf('Slice %d->%d at [%d,%d]: %s -> %s\n', ...
                     slice_idx, slice_idx+1, i, j, currentStatus, nextStatus);
                 
                 % If either is blocked, analyze why
                 if ~currentTraversable
                     obj.analyzeBlockedLogic(i, j, slice_idx, sprintf('Slice%d', slice_idx));
                 end
                 if ~nextTraversable
                     obj.analyzeBlockedLogic(i, j, slice_idx+1, sprintf('Slice%d', slice_idx+1));
                 end
                 
                 if currentTraversable && nextTraversable
                    % Check if gateway condition is met
                    currentCost = currentSlice.cT(i, j);
                    nextCost = nextSlice.cT(i, j);
                    
                    fprintf('  Costs: %.2f -> %.2f\n', currentCost, nextCost);
                    
                    if nextCost <= currentCost
                        fprintf('  Gateway: OPEN (cost allows transition)\n');
                    else
                        fprintf('  Gateway: RESTRICTED (cost increase)\n');
                    end
                end
            end
            
            % Check for alternative paths
            fprintf('\n--- Alternative Path Search ---\n');
            fprintf('Searching for traversable corridors...\n');
            
            for slice_idx = 1:length(obj.slices)
                slice = obj.slices{slice_idx};
                fprintf('Slice %d:\n', slice_idx);
                
                % Count traversable cells in different regions
                regions = struct(...
                    'minRow', {1, 11, 21, 31}, ...
                    'maxRow', {10, 20, 30, obj.gridSize(1)}, ...
                    'name', {'Bottom', 'Lower-Mid', 'Upper-Mid', 'Top'});
                
                for r = 1:length(regions)
                    minRow = regions(r).minRow;
                    maxRow = min(regions(r).maxRow, obj.gridSize(1));
                    regionName = regions(r).name;
                    
                    traversableInRegion = 0;
                    totalInRegion = 0;
                    
                    for i = minRow:maxRow
                        for j = 1:obj.gridSize(2)
                            totalInRegion = totalInRegion + 1;
                            if obj.isTraversable(i, j, slice_idx)
                                traversableInRegion = traversableInRegion + 1;
                            end
                        end
                    end
                    
                    if totalInRegion > 0
                        ratio = (traversableInRegion / totalInRegion) * 100;
                        fprintf('  %s region: %.1f%% traversable (%d/%d)\n', ...
                            regionName, ratio, traversableInRegion, totalInRegion);
                    end
                end
            end
        end
        
        function analyzeBlockedLogic(obj, i, j, sliceIdx, label)
            % Detailed analysis of why a specific grid position is blocked
            fprintf('\n=== BLOCKED LOGIC ANALYSIS for %s [%d,%d,%d] ===\n', label, i, j, sliceIdx);
            
            if sliceIdx < 1 || sliceIdx > length(obj.slices)
                fprintf('ERROR: Invalid slice index\n');
                return;
            end
            
            slice = obj.slices{sliceIdx};
            
            % Step 1: Check bounds
            fprintf('1. BOUNDS CHECK:\n');
            if i < 1 || i > obj.gridSize(1) || j < 1 || j > obj.gridSize(2)
                fprintf('   FAIL: Out of grid bounds [%d,%d] vs [%d,%d]\n', ...
                    i, j, obj.gridSize(1), obj.gridSize(2));
                return;
            else
                fprintf('   PASS: Within grid bounds\n');
            end
            
            % Step 2: Check basic data availability
            fprintf('2. DATA AVAILABILITY:\n');
            eG = slice.eG(i,j);
            eC = slice.eC(i,j);
            cT = slice.cT(i,j);
            
            fprintf('   Ground height (eG): %.3f\n', eG);
            fprintf('   Ceiling height (eC): %.3f\n', eC);
            fprintf('   Traversability cost (cT): %.3f\n', cT);
            
            if isnan(eG) || isnan(eC)
                fprintf('   FAIL: Missing ground/ceiling data (NaN)\n');
                return;
            else
                fprintf('   PASS: Valid ground/ceiling data\n');
            end
            
            % Step 3: Check traversability cost
            fprintf('3. TRAVERSABILITY COST CHECK:\n');
            fprintf('   Cost: %.3f, Barrier threshold: %.3f\n', cT, obj.costParams.c_B);
            
            if isnan(cT)
                fprintf('   FAIL: Traversability cost is NaN\n');
                
                % Analyze why cT is NaN - check intermediate calculations
                fprintf('   INVESTIGATING WHY cT = NaN:\n');
                
                % Check clearance
                clearance = eC - eG;
                fprintf('     Clearance (dI): %.3f vs d_min: %.3f\n', clearance, obj.robotParams.d_min);
                
                if clearance < obj.robotParams.d_min
                    fprintf('     -> cI = c_B (insufficient clearance)\n');
                else
                    fprintf('     -> cI calculation passed clearance test\n');
                end
                
                % Check support distance
                support = slice.z - eG;
                supportThreshold = 2.0 * obj.ds;
                fprintf('     Support distance: %.3f vs threshold: %.3f\n', support, supportThreshold);
                
                if support > supportThreshold
                    fprintf('     -> REASON: Support too far (%.3f > %.3f) -> cT set to NaN\n', ...
                        support, supportThreshold);
                    return;
                else
                    fprintf('     -> Support distance check passed\n');
                end
                
                % Check if gradients cause issues
                [gx, gy] = obj.calculateGradients(slice.eG);
                gradMag = sqrt(gx(i,j)^2 + gy(i,j)^2);
                fprintf('     Gradient magnitude: %.3f vs theta_s: %.3f\n', gradMag, obj.costParams.theta_s);
                
                return;
                
            elseif cT >= obj.costParams.c_B
                fprintf('   FAIL: Cost too high (%.3f >= %.3f)\n', cT, obj.costParams.c_B);
                return;
            else
                fprintf('   PASS: Cost acceptable (%.3f < %.3f)\n', cT, obj.costParams.c_B);
            end
            
            % Step 4: Check clearance
            fprintf('4. CLEARANCE CHECK:\n');
            clearance = eC - eG;
            fprintf('   Clearance: %.3f vs d_min: %.3f\n', clearance, obj.robotParams.d_min);
            
            if clearance < obj.robotParams.d_min
                fprintf('   FAIL: Insufficient clearance\n');
                return;
            else
                fprintf('   PASS: Sufficient clearance\n');
            end
            
            % Step 5: Check support distance
            fprintf('5. SUPPORT DISTANCE CHECK:\n');
            support = slice.z - eG;
            supportThreshold = 2.0 * obj.ds;
            fprintf('   Support: %.3f vs threshold: %.3f\n', support, supportThreshold);
            
            if support > supportThreshold
                fprintf('   FAIL: Support too far\n');
                return;
            else
                fprintf('   PASS: Support within range\n');
            end
            
            % Step 6: Final result
            fprintf('6. FINAL RESULT:\n');
            isTraversable = obj.isTraversable(i, j, sliceIdx);
            if isTraversable
                fprintf('   RESULT: TRAVERSABLE\n');
            else
                fprintf('   RESULT: BLOCKED (unknown reason - check isTraversable method)\n');
            end
        end
        
        function visualizeTrajectoryInScene(obj, originalPath, optimizedPath, sceneParams)
            % Visualize optimized trajectory in the original stair scene
            
            figure('Name', 'PCT Trajectory in Stair Scene', 'Position', [100, 100, 1400, 900]);
            
            % Main 3D view
            subplot(2, 3, [1, 2, 4, 5]);
            hold on;
            
            % 1. Draw the stair scene structure
            if exist('sceneParams', 'var')
                obj.drawStairStructure(sceneParams);
            end
            
            % 2. Show tomogram slices as semi-transparent surfaces
            obj.visualizeSlices();
            alpha(0.3);  % Make slices semi-transparent
            
            % 3. Draw the original A* path
            if ~isempty(originalPath)
                % Convert grid indices to world coordinates for original path
                origWorldPath = obj.convertPathToWorld(originalPath);
                plot3(origWorldPath(:,1), origWorldPath(:,2), origWorldPath(:,3), ...
                    'r--', 'LineWidth', 2, 'DisplayName', 'A* Path');
                plot3(origWorldPath(1,1), origWorldPath(1,2), origWorldPath(1,3), ...
                    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Start');
                plot3(origWorldPath(end,1), origWorldPath(end,2), origWorldPath(end,3), ...
                    'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');
            end
            
            % 4. Draw the optimized trajectory
            if ~isempty(optimizedPath)
                plot3(optimizedPath(:,1), optimizedPath(:,2), optimizedPath(:,3), ...
                    'g-', 'LineWidth', 3, 'DisplayName', 'Optimized Trajectory');
                
                % Mark key points along trajectory
                numMarkers = min(10, size(optimizedPath, 1));
                markerIndices = round(linspace(1, size(optimizedPath, 1), numMarkers));
                plot3(optimizedPath(markerIndices,1), optimizedPath(markerIndices,2), optimizedPath(markerIndices,3), ...
                    'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');
            end
            
            % Enhance visualization
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('PCT Navigation in Stair Scene');
            legend('Location', 'best');
            grid on;
            axis equal;
            view(45, 30);
            
            % Add lighting for better 3D perception
            lighting gouraud;
            light('Position', [1, 1, 1]);
            
            % Side view (XZ plane)
            subplot(2, 3, 3);
            hold on;
            if ~isempty(originalPath)
                origWorldPath = obj.convertPathToWorld(originalPath);
                plot(origWorldPath(:,2), origWorldPath(:,3), 'r--', 'LineWidth', 2, 'DisplayName', 'A* Path');
            end
            if ~isempty(optimizedPath)
                plot(optimizedPath(:,2), optimizedPath(:,3), 'g-', 'LineWidth', 3, 'DisplayName', 'Optimized');
            end
            
            % Draw stair profile
            if exist('sceneParams', 'var')
                obj.drawStairProfile(sceneParams);
            end
            
            xlabel('Y (m)'); ylabel('Z (m)');
            title('Side View (Profile)');
            legend('Location', 'best');
            grid on;
            
            % Top view (XY plane)
            subplot(2, 3, 6);
            hold on;
            if ~isempty(originalPath)
                origWorldPath = obj.convertPathToWorld(originalPath);
                plot(origWorldPath(:,1), origWorldPath(:,2), 'r--', 'LineWidth', 2, 'DisplayName', 'A* Path');
            end
            if ~isempty(optimizedPath)
                plot(optimizedPath(:,1), optimizedPath(:,2), 'g-', 'LineWidth', 3, 'DisplayName', 'Optimized');
            end
            
            % Draw stair footprint
            if exist('sceneParams', 'var')
                obj.drawStairFootprint(sceneParams);
            end
            
            xlabel('X (m)'); ylabel('Y (m)');
            title('Top View (Footprint)');
            legend('Location', 'best');
            grid on;
            axis equal;
            
            % Add overall title with trajectory statistics
            if ~isempty(originalPath) && ~isempty(optimizedPath)
                origWorldPath = obj.convertPathToWorld(originalPath);
                origDist = sum(sqrt(sum(diff(origWorldPath).^2, 2)));
                optDist = sum(sqrt(sum(diff(optimizedPath).^2, 2)));
                heightGain = optimizedPath(end,3) - optimizedPath(1,3);
                
                sgtitle(sprintf('PCT Quadruped Navigation\nPath Length: %.2fm → %.2fm (%.1f%% reduction), Height Gain: %.2fm', ...
                    origDist, optDist, (1-optDist/origDist)*100, heightGain));
            end
            
            hold off;
        end
        
        function worldPath = convertPathToWorld(obj, gridPath)
            % Convert grid path indices to world coordinates
            worldPath = zeros(size(gridPath, 1), 3);
            
            for i = 1:size(gridPath, 1)
                gridI = gridPath(i, 1);
                gridJ = gridPath(i, 2);
                sliceIdx = gridPath(i, 3);
                
                % Convert grid to world coordinates
                worldPath(i, 1) = obj.minX + (gridJ - 1) * obj.rg;  % X
                worldPath(i, 2) = obj.minY + (gridI - 1) * obj.rg;  % Y
                
                % Get Z from slice height
                if sliceIdx >= 1 && sliceIdx <= length(obj.slices)
                    worldPath(i, 3) = obj.slices{sliceIdx}.z;
                else
                    worldPath(i, 3) = 0;
                end
            end
        end
        
        function drawStairStructure(obj, sceneParams)
            % Draw 3D stair structure for context
            if ~isfield(sceneParams, 'STEP_H') || ~isfield(sceneParams, 'STEP_D') || ~isfield(sceneParams, 'STEP_W')
                return;
            end
            
            stepH = sceneParams.STEP_H;
            stepD = sceneParams.STEP_D;
            stepW = sceneParams.STEP_W;
            N = sceneParams.N;
            
            % Draw ascending stairs
            for i = 1:N
                x = [0, stepW, stepW, 0, 0];
                y = [(i-1)*stepD, (i-1)*stepD, i*stepD, i*stepD, (i-1)*stepD];
                z_bottom = [(i-1)*stepH, (i-1)*stepH, (i-1)*stepH, (i-1)*stepH, (i-1)*stepH];
                z_top = [i*stepH, i*stepH, i*stepH, i*stepH, i*stepH];
                
                % Draw step surface
                fill3(x, y, z_top, [0.8, 0.8, 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
                
                % Draw step riser
                if i > 1
                    y_riser = [(i-1)*stepD, (i-1)*stepD, (i-1)*stepD, (i-1)*stepD, (i-1)*stepD];
                    x_riser = [0, stepW, stepW, 0, 0];
                    z_riser = [(i-1)*stepH, (i-1)*stepH, i*stepH, i*stepH, (i-1)*stepH];
                    fill3(x_riser, y_riser, z_riser, [0.7, 0.7, 0.7], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
                end
            end
            
            % Draw descending stairs
            for i = 1:N
                x = [0, stepW, stepW, 0, 0];
                y = [N*stepD + i*stepD, N*stepD + i*stepD, N*stepD + (i+1)*stepD, N*stepD + (i+1)*stepD, N*stepD + i*stepD];
                z_level = (N-i)*stepH;
                z_bottom = [z_level, z_level, z_level, z_level, z_level];
                z_top = [z_level, z_level, z_level, z_level, z_level];
                
                % Draw step surface
                fill3(x, y, z_top, [0.8, 0.8, 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'k');
            end
        end
        
        function drawStairProfile(obj, sceneParams)
            % Draw stair profile in side view
            if ~isfield(sceneParams, 'STEP_H') || ~isfield(sceneParams, 'STEP_D')
                return;
            end
            
            stepH = sceneParams.STEP_H;
            stepD = sceneParams.STEP_D;
            N = sceneParams.N;
            
            % Ascending stairs profile
            y_profile = [];
            z_profile = [];
            
            for i = 0:N
                y_profile = [y_profile, i*stepD, i*stepD];
                z_profile = [z_profile, i*stepH, i*stepH];
                if i < N
                    y_profile = [y_profile, (i+1)*stepD];
                    z_profile = [z_profile, i*stepH];
                end
            end
            
            % Descending stairs profile
            for i = 1:N
                y_val = N*stepD + i*stepD;
                z_val = (N-i)*stepH;
                y_profile = [y_profile, y_val];
                z_profile = [z_profile, z_val];
            end
            
            plot(y_profile, z_profile, 'k-', 'LineWidth', 2, 'DisplayName', 'Stair Profile');
        end
        
        function drawStairFootprint(obj, sceneParams)
            % Draw stair footprint in top view
            if ~isfield(sceneParams, 'STEP_D') || ~isfield(sceneParams, 'STEP_W')
                return;
            end
            
            stepD = sceneParams.STEP_D;
            stepW = sceneParams.STEP_W;
            N = sceneParams.N;
            
            % Draw stair outline
            totalLength = 2*N*stepD;
            x_outline = [0, stepW, stepW, 0, 0];
            y_outline = [0, 0, totalLength, totalLength, 0];
            
            plot(x_outline, y_outline, 'k-', 'LineWidth', 2, 'DisplayName', 'Stair Outline');
            
            % Draw step divisions
            for i = 1:2*N-1
                y_line = i * stepD;
                plot([0, stepW], [y_line, y_line], 'k--', 'LineWidth', 1);
            end
        end
    end
end 