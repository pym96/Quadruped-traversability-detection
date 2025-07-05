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
            obj.robotParams.d_min = 0.3;    % Minimum robot height
            obj.robotParams.d_ref = 0.40;    % Reference (normal) robot height
            
            obj.costParams.c_B = 50;        % Barrier/obstacle cost
            obj.costParams.alpha_d = 5;     % Height adjustment cost factor
            obj.costParams.theta_b = 20;   % Obstacle boundary threshold
            obj.costParams.theta_s = 10;   % Flat surface threshold
            obj.costParams.theta_p = 0.4;   % Safe neighbor ratio threshold (increased from 0.2)
            obj.costParams.alpha_s = 0.5;     % Surface cost factor
            obj.costParams.alpha_b = 1;     % Boundary cost factor (decreased from 2)
            obj.costParams.r_g = 0.15;       % Grid resolution
            obj.costParams.d_inf = 0.4;     % Inflation distance
            obj.costParams.d_sm = 0.4;      % Safety margin
            
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
                dI(isnan(dI)) = obj.costParams.c_B;  % Mark invalid regions as obstacles
                
                % Calculate height-based cost
                cI = zeros(size(dI));
                cI(dI < obj.robotParams.d_min) = obj.costParams.c_B;  % Too low to pass
                adjustable = dI >= obj.robotParams.d_min & dI <= obj.robotParams.d_ref;
                cI(adjustable) = max(0, obj.costParams.alpha_d * (obj.robotParams.d_ref - dI(adjustable)));
                
                % Step 2: Analyze ground conditions
                [rows, cols] = size(eG);
                
                % Calculate physical distances between grid points
                dx = obj.rg;  % Grid resolution in x direction
                dy = obj.rg;  % Grid resolution in y direction
                
                % Calculate gradients using physical distances
                gx = zeros(size(eG));
                gy = zeros(size(eG));
                
                % X direction gradient (using central difference)
                for i = 1:rows
                    for j = 2:cols-1
                        gx(i,j) = (eG(i,j+1) - eG(i,j-1)) / (2*dx);
                    end
                    % Forward/backward difference at boundaries
                    if cols > 1
                        gx(i,1) = (eG(i,2) - eG(i,1)) / dx;
                        gx(i,cols) = (eG(i,cols) - eG(i,cols-1)) / dx;
                    end
                end
                
                % Y direction gradient (using central difference)
                for j = 1:cols
                    for i = 2:rows-1
                        gy(i,j) = (eG(i+1,j) - eG(i-1,j)) / (2*dy);
                    end
                    % Forward/backward difference at boundaries
                    if rows > 1
                        gy(1,j) = (eG(2,j) - eG(1,j)) / dy;
                        gy(rows,j) = (eG(rows,j) - eG(rows-1,j)) / dy;
                    end
                end
                
                % Calculate gradient metrics
                mxy = max(abs(gx), abs(gy));  % Maximum directional gradient
                mgrad = sqrt(gx.^2 + gy.^2);  % Total gradient magnitude
                
                % Calculate ground-based cost
                cG = zeros(size(eG));
                
                % For each grid cell
                for i = 2:rows-1
                    for j = 2:cols-1
                        % Get 3x3 neighborhood
                        patch = mgrad(i-1:i+1, j-1:j+1);
                        % Calculate ps: proportion of neighboring grids with m^grad < theta_s
                        ps = sum(patch(:) < obj.costParams.theta_s) / numel(patch);
                        
                        % Strictly follow formula (4)
                        if ps > obj.costParams.theta_p
                            % If enough neighbors are smooth, use the boundary cost formula
                            cG(i,j) = obj.costParams.alpha_b * (mxy(i,j) / obj.costParams.theta_b)^2;
                        else
                            % Otherwise, mark as barrier
                            cG(i,j) = obj.costParams.c_B;
                        end
                    end
                end
                
                % Mark edges as barriers
                cG(1,:) = obj.costParams.c_B;
                cG(end,:) = obj.costParams.c_B;
                cG(:,1) = obj.costParams.c_B;
                cG(:,end) = obj.costParams.c_B;
                
                % Step 3: Combine costs and apply inflation
                cinit = min(obj.costParams.c_B, cI + cG);
                
                % Create inflation kernel
                kernel_size = 5;
                [X, Y] = meshgrid(-2:2, -2:2);
                dist = sqrt(X.^2 + Y.^2) * obj.costParams.r_g;
                K = max(0, min(1 - (dist - obj.costParams.d_inf) / (obj.costParams.d_sm - obj.costParams.r_g), 1));
                K = K / max(K(:));  % Normalize kernel
                
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
                slice.eG = eG;  % Now storing absolute heights
                slice.eC = eC;  % Now storing absolute heights
                slice.cT = cT;
                obj.slices{k+1} = slice;
            end
            
            % Remove empty slices
            valid_slices = false(length(obj.slices), 1);
            for k = 1:length(obj.slices)
                if ~all(isnan(obj.slices{k}.eG(:))) || ~all(isnan(obj.slices{k}.eC(:)))
                    valid_slices(k) = true;
                end
            end
            obj.slices = obj.slices(valid_slices);
            obj.N = length(obj.slices) - 1;
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
                    traversable = ground_valid & (slice.cT < obj.costParams.c_B * 0.5);
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