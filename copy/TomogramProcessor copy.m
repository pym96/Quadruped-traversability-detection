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
            obj.robotParams.d_min = 0.4;    % Minimum robot height
            obj.robotParams.d_ref = 0.50;    % Reference (normal) robot height
            
            obj.costParams.c_B = 100;        % Barrier/obstacle cost
            obj.costParams.alpha_d = 5;     % Height adjustment cost factor
            obj.costParams.theta_b = 1.5;   % Obstacle boundary threshold
            obj.costParams.theta_s = 0.2;   % Flat surface threshold
            obj.costParams.theta_p = 0.5;   % Safe neighbor ratio threshold (increased from 0.2)
            obj.costParams.alpha_s = 0.5;     % Surface cost factor
            obj.costParams.alpha_b = 1;     % Boundary cost factor (decreased from 2)
            obj.costParams.r_g = 0.15;       % Grid resolution
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
            obj.zMin = min(points(:,3));
            obj.zMax = max(points(:,3));
            obj.N = ceil((obj.zMax - obj.zMin) / obj.ds);
            
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
        end
        
        function obj = processTomograms(obj)
            % Process point cloud into tomogram slices
            obj.slices = cell(obj.N+1, 1);
            
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
            
            % Process each slice
            for k = 0:obj.N
                plane_z = obj.zMin + k * obj.ds;
                
                % Initialize height maps
                eG = nan(obj.gridSize);  % Ground height map
                eC = nan(obj.gridSize);  % Ceiling height map
                
                % Process points for this slice
                for u = 1:size(obj.pointCloud, 1)
                    x = obj.pointCloud(u,1);
                    y = obj.pointCloud(u,2);
                    z = obj.pointCloud(u,3);
                    
                    % Convert to grid coordinates
                    i = max(1, min(obj.gridSize(1), ceil((y-min(obj.pointCloud(:,2)))/obj.rg)));
                    j = max(1, min(obj.gridSize(2), ceil((x-min(obj.pointCloud(:,1)))/obj.rg)));
                    
                    if z >= plane_z
                        % Update ground height (store absolute height)
                        if isnan(eG(i,j)) || z < eG(i,j)
                            eG(i,j) = z;
                        end
                    else
                        % Update ceiling height (store absolute height)
                        if isnan(eC(i,j)) || z > eC(i,j)
                            eC(i,j) = z;
                        end
                    end
                end
                
                % Step 1: Calculate height interval cost
                dI = eC - eG;  % Height interval
                
                % Calculate height-based cost
                cI = zeros(size(dI));
                cI(isnan(dI)) = obj.costParams.c_B;  % Mark invalid regions as obstacles
                cI(dI < obj.robotParams.d_min) = obj.costParams.c_B;  % Too low to pass
                adjustable = dI >= obj.robotParams.d_min & dI <= obj.robotParams.d_ref;
                cI(adjustable) = max(0, obj.costParams.alpha_d * (obj.robotParams.d_ref - dI(adjustable)));
                
                % Step 2: Analyze ground conditions
                [rows, cols] = size(eG);
                
                % Calculate gradients using physical distances
                gx = zeros(size(eG));
                gy = zeros(size(eG));
                
                % X direction gradient (using central difference)
                for i = 1:rows
                    for j = 2:cols-1
                        if ~isnan(eG(i,j+1)) && ~isnan(eG(i,j-1))
                            gx(i,j) = (eG(i,j+1) - eG(i,j-1)) / (2*obj.rg);
                        end
                    end
                end
                
                % Y direction gradient (using central difference)
                for j = 1:cols
                    for i = 2:rows-1
                        if ~isnan(eG(i+1,j)) && ~isnan(eG(i-1,j))
                            gy(i,j) = (eG(i+1,j) - eG(i-1,j)) / (2*obj.rg);
                        end
                    end
                end
                
                % Calculate gradient metrics
                mxy = max(abs(gx), abs(gy));  % Maximum directional gradient
                mgrad = sqrt(gx.^2 + gy.^2);  % Total gradient magnitude
                
                % Calculate ground-based cost
                cG = zeros(size(eG));
                cG(isnan(eG)) = obj.costParams.c_B;  % Mark invalid regions as obstacles
                
                % For each valid grid cell
                for i = 2:rows-1
                    for j = 2:cols-1
                        if ~isnan(eG(i,j))
                            if mgrad(i,j) < obj.costParams.theta_s
                                % Flat surface cost
                                cG(i,j) = obj.costParams.alpha_s * (mgrad(i,j) / obj.costParams.theta_s^2);
                            else
                                % Get 3x3 neighborhood
                                patch = mgrad(i-1:i+1, j-1:j+1);
                                valid_patch = ~isnan(patch);
                                if any(valid_patch(:))
                                    % Calculate ps: proportion of neighboring grids with m^grad < theta_s
                                    ps = sum(patch(:) < obj.costParams.theta_s & valid_patch(:)) / sum(valid_patch(:));
                                    
                                    if mxy(i,j) > obj.costParams.theta_b
                                        cG(i,j) = obj.costParams.c_B;  % Obstacle boundary
                                    elseif ps > obj.costParams.theta_p
                                        % Safe step/ramp
                                        cG(i,j) = obj.costParams.alpha_b * (mxy(i,j) / obj.costParams.theta_b^2);
                                    else
                                        cG(i,j) = obj.costParams.c_B;  % Unsafe region
                                    end
                                else
                                    cG(i,j) = obj.costParams.c_B;
                                end
                            end
                        end
                    end
                end
                
                % Step 3: Combine costs
                cinit = min(obj.costParams.c_B, cI + cG);
                
                % Add center line preference
                [rows, cols] = size(cinit);
                centerX = cols/2;
                [X, Y] = meshgrid(1:cols, 1:rows);
                centerDist = abs(X - centerX) * obj.rg;
                
                % 使用二次函数增加边缘成本
                edgePenalty = (centerDist / (cols/4 * obj.rg)).^2;  % 使用二次函数
                edgePenalty = edgePenalty * obj.costParams.c_B * 0.3;  % 边缘惩罚最大为障碍成本的30%
                cinit = cinit + edgePenalty;
                cinit = min(cinit, obj.costParams.c_B);  % 确保不超过障碍成本
                
                % Print statistics for key slices
                if mod(k, 5) == 0 || k == 0 || k == obj.N
                    fprintf('\n=== Slice %d/%d (z=%.2fm) ===\n', k, obj.N, plane_z);
                    
                    % Height interval statistics
                    validDI = dI(~isnan(dI));
                    if ~isempty(validDI)
                        validDI = validDI(:);  % Ensure column vector
                        fprintf('Height Interval (dI):\n');
                        fprintf('  Mean: %.2f m, Max: %.2f m, Min: %.2f m\n', ...
                            mean(validDI), ...
                            max(validDI), ...
                            min(validDI));
                    else
                        fprintf('Height Interval (dI): No valid measurements\n');
                    end
                    
                    % Gradient statistics
                    valid_mgrad = mgrad(~isnan(mgrad));
                    if ~isempty(valid_mgrad)
                        valid_mgrad = valid_mgrad(:);  % Ensure column vector
                        valid_mxy = mxy(~isnan(mxy));
                        fprintf('Ground Gradients:\n');
                        fprintf('  Mean mgrad: %.2f, Max mxy: %.2f\n', ...
                            mean(valid_mgrad), ...
                            max(valid_mxy(:)));
                    end
                    
                    % Cost statistics
                    valid_cinit = cinit(~isnan(cinit));
                    if ~isempty(valid_cinit)
                        traversable = sum(valid_cinit < obj.costParams.c_B);
                        total = numel(valid_cinit);
                        fprintf('Traversability:\n');
                        fprintf('  Traversable cells: %d (%.1f%%)\n', ...
                            traversable, 100 * traversable / total);
                    end
                end
                
                % Create inflation kernel
                kernel_size = 5;
                [X, Y] = meshgrid(-2:2, -2:2);
                dist = sqrt(X.^2 + Y.^2) * obj.costParams.r_g;
                K = max(0, min(1 - (dist - obj.costParams.d_inf) / ...
                    (obj.costParams.d_sm - obj.costParams.r_g), 1));
                
                % Apply inflation using sliding window
                cT = zeros(size(cinit));
                for i = 3:rows-2
                    for j = 3:cols-2
                        patch = cinit(i-2:i+2, j-2:j+2);
                        cT(i,j) = max(max(K .* patch));
                    end
                end
                
                % Store slice data
                slice.z = plane_z;
                slice.eG = eG;
                slice.eC = eC;
                slice.cT = cT;
                slice.cI = cI;
                slice.cG = cG;
                obj.slices{k+1} = slice;
            end
            
            % After processing all slices, apply simplification
            fprintf('\nSimplifying tomogram slices...\n');
            obj = obj.simplifyTomograms();
        end
        
        function obj = simplifyTomograms(obj)
            % Simplify tomogram slices by removing redundant ones
            % Parameters for simplification
            threshold_height = 0.01;  % 高度变化阈值 (m)
            threshold_cost = -1;      % 成本降低阈值
            
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
                    
                    % Plot traversable areas
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
                             'FaceAlpha', 0.8);
                    end
                    
                    % Plot blocked areas
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
                             'FaceAlpha', 0.8);
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
            openSet = java.util.PriorityQueue(@(a,b) a(5) - b(5));  % Sort by fScore
            openSet.add([startI, startJ, startK, 0, obj.heuristic([startI, startJ, startK], [endI, endJ, endK])]);
            
            % Use string keys for the maps
            cameFrom = containers.Map('KeyType', 'char', 'ValueType', 'any');
            gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            fScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            % Initialize scores
            startKey = obj.nodeToKey([startI, startJ, startK]);
            gScore(startKey) = 0;
            fScore(startKey) = obj.heuristic([startI, startJ, startK], [endI, endJ, endK]);
            
            % A* main loop
            while ~openSet.isEmpty()
                current = openSet.poll();
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
                        openSet.add([neighborNode, tentative_gScore, f]);
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
    end
end 