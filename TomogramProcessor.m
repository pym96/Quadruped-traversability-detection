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
            obj.robotParams.d_min = 0.2;    % Minimum robot height
            obj.robotParams.d_ref = 0.50;    % Reference (normal) robot height
            
            obj.costParams.c_B = 50;        % Barrier/obstacle cost
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
                validDI = dI(~isnan(dI));
                
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
                
                % Print statistics for key slices
                fprintf('\n=== Slice %d/%d (z=%.2fm) ===\n', k, obj.N, plane_z);
                
                % Height interval statistics
                if ~isempty(validDI)
                    fprintf('Height Interval (dI):\n');
                    fprintf('  Mean: %.2f m, Max: %.2f m, Min: %.2f m\n', ...
                        mean(validDI), ...
                        max(validDI), ...
                        min(validDI));
                end
                
                % Gradient statistics
                valid_mgrad = mgrad(~isnan(mgrad));
                if ~isempty(valid_mgrad)
                    fprintf('Ground Gradients:\n');
                    fprintf('  Mean mgrad: %.2f, Max mxy: %.2f\n', ...
                        mean(valid_mgrad), max(mxy(:)));
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
            
            % Remove empty slices and print summary
            valid_slices = false(length(obj.slices), 1);
            total_traversable = 0;
            total_cells = 0;
            
            for k = 1:length(obj.slices)
                if ~all(isnan(obj.slices{k}.eG(:))) || ~all(isnan(obj.slices{k}.eC(:)))
                    valid_slices(k) = true;
                    valid_costs = obj.slices{k}.cT(~isnan(obj.slices{k}.cT));
                    total_traversable = total_traversable + sum(valid_costs < obj.costParams.c_B);
                    total_cells = total_cells + numel(valid_costs);
                end
            end
            
            obj.slices = obj.slices(valid_slices);
            obj.N = length(obj.slices) - 1;
            
            fprintf('\nFinal Statistics:\n');
            fprintf('Valid slices: %d\n', obj.N + 1);
            fprintf('Overall traversability: %.1f%% (%d/%d cells)\n', ...
                100 * total_traversable / total_cells, ...
                total_traversable, total_cells);
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
    end
end 