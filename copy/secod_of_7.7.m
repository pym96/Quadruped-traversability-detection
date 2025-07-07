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
            obj.robotParams.d_min = 0.35;    % Minimum robot height
            obj.robotParams.d_ref = 0.55;    % Reference (normal) robot height
            % Safety margin for vertical transitions
            obj.robotParams.d_sm = 0.2;      % Same as costParams.d_sm
            % Maximum acceptable ground slope (same as theta_s threshold)
            obj.robotParams.theta_max = 0.5;
            
            obj.costParams.c_B = 92;        % Barrier/obstacle cost
            obj.costParams.alpha_d = 0.4;     % Height adjustment cost factor
            obj.costParams.theta_b = 10;   % Obstacle boundary threshold
            obj.costParams.theta_s = 0.7;   % Flat surface threshold
            obj.costParams.theta_p = 0.2;   % Safe neighbor ratio threshold
            obj.costParams.alpha_s = 0.1;   % Surface cost factor
            obj.costParams.alpha_b = 0.1;     % Boundary cost factor
            obj.costParams.r_g = 0.15;      % Grid resolution
            obj.costParams.d_inf = 0.2;     % Inflation distance
            obj.costParams.d_sm = 0.2;      % Safety margin
            
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
            obj.zMin = floor(rawZmin / obj.ds) * obj.ds;  % align to nearest slice plane
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
                    
                    % If point is below or at slice height, update ground (max)
                    if z <= slice_z
                        slice.eG(i,j) = max(z, slice.eG(i,j));
                    % If point is above slice height, update ceiling (min)
                    else
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
            cInit = max(cI, 0.5*cI + 0.5*cG);
            cT = min(obj.costParams.c_B, cInit);

            % -----------------------------------------------------------------
            % Additional invalidation: if the ground that supports this slice
            % is farther than half a slice below the current plane, the robot
            % cannot actually stand here.  Mark these cells invalid (NaN) so
            % they will neither be considered traversable nor be rendered
            % as blue patches in the visualization.
            % -----------------------------------------------------------------
            supportMap = slice.z - slice.eG;          % vertical distance to ground
            farSupportMask = supportMap > 0.8*obj.ds;    % out of supporting range
            cT(farSupportMask) = NaN;
            cI(farSupportMask) = NaN;
            cG(farSupportMask) = NaN;
            cInit(farSupportMask) = NaN;
        end
        
        function [cG, gx, gy, mxy] = calculateGroundCost(obj, eG)
            % Calculate ground-based cost
            [rows, cols] = size(eG);
            cG = nan(rows, cols);
            cG(isnan(eG)) = obj.costParams.c_B;
            
            % Calculate gradients
            [gx, gy] = obj.calculateGradients(eG);
            mxy = max(abs(gx), abs(gy));
            mgrad = sqrt(gx.^2 + gy.^2);
            
            % Initialize gradient cost to barrier for invalid cells
            cG(:) = obj.costParams.c_B;
            
            for i = 2:rows-1
                for j = 2:cols-1
                    if ~isnan(eG(i,j))
                        if mgrad(i,j) < obj.costParams.theta_s
                            cG(i,j) = obj.costParams.alpha_s * (mgrad(i,j) / obj.costParams.theta_s^2);
                        else
                            patch = mgrad(i-1:i+1, j-1:j+1);
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
            % Simplify tomogram slices by removing redundant ones
            % Parameters for simplification
            threshold_height = 0.0;  % 高度变化阈值 (m)
            threshold_cost = 0.0;      % 成本降低阈值
            
            % 记录原始切片数
            original_count = length(obj.slices);
            valid_slices = true(original_count, 1);
            
            % 遍历所有中间切片（保留首尾切片）
            for k = 2:length(obj.slices)-1
                current_slice = obj.slices{k};
                prev_slice = obj.slices{k-1};
                next_slice = obj.slices{k+1};
                
                % 1. 找出当前切片的可通行网格
                Mk = current_slice.cT < obj.costParams.c_B;
                Mk_prev = prev_slice.cT < obj.costParams.c_B;
                Mk_next = next_slice.cT < obj.costParams.c_B;
                
                % 2. 检查是否存在唯一网格
                [rows, cols] = size(current_slice.cT);
                has_unique_grid = false;
                
                for i = 1:rows
                    for j = 1:cols
                        if ~Mk(i,j)
                            continue;  % 跳过不可通行网格
                        end
                        
                        % 检查高度和成本的唯一性（Eq. 8 和 Eq. 9）
                        height_unique_prev = (current_slice.eG(i,j) - prev_slice.eG(i,j)) > threshold_height;
                        height_unique_next = (next_slice.eG(i,j) - current_slice.eG(i,j)) > threshold_height;
                        
                        cost_unique_prev = current_slice.cT(i,j) < (prev_slice.cT(i,j) + threshold_cost);
                        cost_unique_next = current_slice.cT(i,j) < (next_slice.cT(i,j) + threshold_cost);
                        
                        if (height_unique_prev || cost_unique_prev) && ...
                           (height_unique_next || cost_unique_next)
                            has_unique_grid = true;
                            break;
                        end
                    end
                    if has_unique_grid
                        break;
                    end
                end
                
                % 3. 如果没有唯一网格，标记为冗余
                if ~has_unique_grid
                    valid_slices(k) = false;
                end
            end
            
            % 4. 移除冗余切片
            obj.slices = obj.slices(valid_slices);
            obj.N = length(obj.slices) - 1;
            
            % 5. 打印简化结果
            fprintf('Tomogram simplified: %d -> %d slices (%.1f%% reduction)\n', ...
                original_count, length(obj.slices), ...
                (1 - length(obj.slices)/original_count) * 100);
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
            % Plan a path from startPos to endPos using A* algorithm
            % startPos and endPos are [x, y, z] in world coordinates
            
            % Convert world coordinates to grid indices
            startX = startPos(1); startY = startPos(2); startZ = startPos(3);
            endX = endPos(1); endY = endPos(2); endZ = endPos(3);
            
            startI = max(1, min(obj.gridSize(1), ceil((startY - obj.minY) / obj.rg)));
            startJ = max(1, min(obj.gridSize(2), ceil((startX - obj.minX) / obj.rg)));
            startK = max(1, min(obj.N+1, ceil((startZ - obj.minZ) / obj.ds)));
            
            endI = max(1, min(obj.gridSize(1), ceil((endY - obj.minY) / obj.rg)));
            endJ = max(1, min(obj.gridSize(2), ceil((endX - obj.minX) / obj.rg)));
            endK = max(1, min(obj.N+1, ceil((endZ - obj.minZ) / obj.ds)));
            
            fprintf('Planning path from [%d,%d,%d] to [%d,%d,%d]\n', ...
                startI, startJ, startK, endI, endJ, endK);
            
            % Check if start and end positions are traversable
            if ~obj.isTraversable(startI, startJ, startK) || ~obj.isTraversable(endI, endJ, endK)
                fprintf('Start or end position is not traversable!\n');
                path = [];
                return;
            end
            
            % Initialize A* data structures
            % Use a simple matrix for the OPEN set: each row is
            % [i, j, k, gScore, fScore]. We always pick the row with the
            % smallest fScore.
            openSet = [startI, startJ, startK, 0, ...
                      obj.heuristic([startI, startJ, startK], [endI, endJ, endK])];
            
            % Use string keys for the maps
            cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');
            gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            fScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            % Initialize scores
            startKey = obj.nodeToKey([startI, startJ, startK]);
            gScore(startKey) = 0;
            fScore(startKey) = obj.heuristic([startI, startJ, startK], [endI, endJ, endK]);
            
            % A* main loop
            while ~isempty(openSet)
                % Extract the node with the lowest fScore
                [~, idxMin] = min(openSet(:, 5));
                current = openSet(idxMin, :);
                % Remove it from the open set
                openSet(idxMin, :) = [];
                currentNode = current(1:3);
                currentKey = obj.nodeToKey(currentNode);
                
                % Check if goal reached
                if all(currentNode == [endI, endJ, endK])
                    path = obj.reconstructPath(cameFrom, currentNode);
                    fprintf('Path found with %d steps!\n', size(path,1));
                    return;
                end
                
                % Get neighbors
                neighbors = obj.getNeighbors(currentNode);
                for n = 1:size(neighbors,1)
                    neighborNode = neighbors(n,:);
                    neighborKey = obj.nodeToKey(neighborNode);
                    
                    % Calculate movement cost
                    moveCost = obj.calculateMovementCost(currentNode, neighborNode);
                    if isinf(moveCost)
                        continue;  % Skip impassable transitions
                    end
                    
                    tentative_gScore = gScore(currentKey) + moveCost;
                    
                    if ~isKey(gScore, neighborKey) || tentative_gScore < gScore(neighborKey)
                        % This path is better than any previous one
                        cameFrom(neighborKey) = currentNode;
                        gScore(neighborKey) = tentative_gScore;
                        f = tentative_gScore + obj.heuristic(neighborNode, [endI, endJ, endK]);
                        fScore(neighborKey) = f;
                        openSet = [openSet; neighborNode, tentative_gScore, f];
                    end
                end
            end
            
            fprintf('No path found!\n');
            path = [];  % No path found
        end
        
        function cost = calculateMovementCost(obj, node1, node2)
            % Calculate the cost of moving from node1 to node2 based on the paper's methodology
            if ~obj.isTraversable(node2(1), node2(2), node2(3))
                cost = inf;
                return;
            end
            
            % Get slice data
            slice1 = obj.slices{node1(3)};
            slice2 = obj.slices{node2(3)};
            
            % Get ground and ceiling heights
            g1 = slice1.eG(node1(1), node1(2));  % Ground height
            c1 = slice1.eC(node1(1), node1(2));  % Ceiling height
            g2 = slice2.eG(node2(1), node2(2));
            c2 = slice2.eC(node2(1), node2(2));
            
            % Calculate interval costs (c') as per paper
            h1 = c1 - g1;  % Height interval for node1
            h2 = c2 - g2;  % Height interval for node2
            
            % Base interval cost calculation
            if h1 < obj.robotParams.d_min || h2 < obj.robotParams.d_min
                cost = inf;  % Non-traversable if clearance too small
                return;
            end
            
            % Interval cost based on clearance
            c_int1 = 1 / (h1 - obj.robotParams.d_min);
            c_int2 = 1 / (h2 - obj.robotParams.d_min);
            
            % Base movement cost (Euclidean distance)
            if node1(3) == node2(3)  % Same slice
                dist = norm([node1(1:2) - node2(1:2)]);
            else  % Different slices - check gateway conditions
                % Verify gateway grid conditions
                if abs(g1 - g2) > obj.robotParams.d_sm  % Ground elevation difference too large
                    cost = inf;
                    return;
                end
                
                % Add vertical transition penalty
                dist = norm([node1(1:2) - node2(1:2)]) * 1.5;  % Higher cost for slice transitions
            end
            
            % Height difference penalty
            heightDiff = abs(g2 - g1);
            heightPenalty = heightDiff * obj.costParams.alpha_s;
            
            % Combine costs as per paper
            cost = dist * (1 + (c_int1 + c_int2)/2) + heightPenalty;
        end
        
        function neighbors = getNeighbors(obj, node)
            % Get valid neighbors with gateway grid consideration
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
                       nj >= 1 && nj <= obj.gridSize(2)
                        neighbors = [neighbors; ni, nj, k];
                    end
                end
            end
            
            % Check for gateway grids in adjacent slices
            if k > 1  % Check lower slice
                slice_current = obj.slices{k};
                slice_lower = obj.slices{k-1};
                
                % Check gateway conditions
                g_current = slice_current.eG(i,j);
                g_lower = slice_lower.eG(i,j);
                
                if abs(g_current - g_lower) <= obj.robotParams.d_sm && ...
                   obj.isTraversable(i, j, k-1)
                    neighbors = [neighbors; i, j, k-1];
                end
            end
            
            if k < length(obj.slices)  % Check upper slice
                slice_current = obj.slices{k};
                slice_upper = obj.slices{k+1};
                
                % Check gateway conditions
                g_current = slice_current.eG(i,j);
                g_upper = slice_upper.eG(i,j);
                
                if abs(g_current - g_upper) <= obj.robotParams.d_sm && ...
                   obj.isTraversable(i, j, k+1)
                    neighbors = [neighbors; i, j, k+1];
                end
            end
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
            if isnan(slice.cT(i,j)) || slice.cT(i,j) >= obj.costParams.c_B
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
            if support > 0.8*obj.ds
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
            % Optimize trajectory using quintic polynomial optimization
            if size(path, 1) < 3
                optimizedPath = path;
                return;
            end
            
            fprintf('Optimizing trajectory with quintic polynomials...\n');
            
            % Time parameterization
            totalDist = sum(sqrt(sum(diff(path).^2, 2)));
            avgSpeed = 0.5;  % m/s
            T = totalDist / avgSpeed;
            t = linspace(0, T, size(path, 1))';
            
            % Initialize optimized trajectory
            optimizedPath = zeros(size(path));
            
            % Optimize each dimension separately
            for dim = 1:3
                % Setup quintic polynomial basis
                beta = @(t) [ones(size(t)), t, t.^2, t.^3, t.^4, t.^5];
                dbeta = @(t) [zeros(size(t)), ones(size(t)), 2*t, 3*t.^2, 4*t.^3, 5*t.^4];
                ddbeta = @(t) [zeros(size(t)), zeros(size(t)), 2*ones(size(t)), 6*t, 12*t.^2, 20*t.^3];
                
                % Setup optimization constraints
                p0 = path(1, dim);    % Initial position
                pf = path(end, dim);  % Final position
                v0 = 0;               % Initial velocity
                vf = 0;               % Final velocity
                a0 = 0;               % Initial acceleration
                af = 0;               % Final acceleration
                
                % Equality constraints (position, velocity, acceleration)
                Aeq = [beta(0); beta(T); dbeta(0); dbeta(T); ddbeta(0); ddbeta(T)];
                beq = [p0; pf; v0; vf; a0; af];
                
                % Cost function: minimize jerk
                costFun = @(sigma) sum((ddbeta(t) * sigma').^2);
                
                % Initial guess
                sigma0 = zeros(6, 1);
                sigma0(1) = p0;
                sigma0(2) = (pf - p0) / T;
                
                % Optimize coefficients
                options = optimoptions('fmincon', 'Display', 'off');
                sigma = fmincon(costFun, sigma0, [], [], Aeq, beq, [], [], [], options);
                
                % Generate optimized trajectory points
                optimizedPath(:, dim) = beta(t) * sigma;
            end
            
            % Apply ceiling constraints and ensure ground clearance
            for i = 1:size(optimizedPath, 1)
                % Find nearest slice
                sliceIdx = obj.findNearestSlice(optimizedPath(i, 3));
                if sliceIdx > 0 && sliceIdx <= length(obj.slices)
                    % Get grid coordinates
                    gridPos = obj.worldToGrid(optimizedPath(i, 1:2));
                    if obj.isValidGridBounds(gridPos)
                        % Get ground and ceiling heights
                        slice = obj.slices{sliceIdx};
                        groundHeight = slice.eG(gridPos(1), gridPos(2));
                        ceilingHeight = slice.eC(gridPos(1), gridPos(2));
                        
                        if ~isnan(groundHeight) && ~isnan(ceilingHeight)
                            % Ensure minimum clearance from ground
                            minHeight = groundHeight + robotParams.d_min;
                            % Ensure maximum clearance from ceiling
                            maxHeight = ceilingHeight - 0.1;  % 10cm safety margin
                            
                            % Clamp height within valid range
                            optimizedPath(i, 3) = min(max(optimizedPath(i, 3), minHeight), maxHeight);
                        end
                    end
                end
            end
            
            fprintf('Trajectory optimized: %d waypoints\n', size(optimizedPath, 1));
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
    end
end 