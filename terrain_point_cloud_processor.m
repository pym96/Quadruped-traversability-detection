classdef terrain_point_cloud_processor
    properties
        pointCloud % Raw point cloud data
        gridSize % Size of each grid cell for discretization
        heightMap % Processed height map
        normalMap % Surface normal map
        traversabilityMap % Map indicating terrain traversability
    end
    
    methods
        function obj = terrain_point_cloud_processor(gridSize)
            % Constructor
            if nargin < 1
                obj.gridSize = 0.2; % Default 20cm grid size (changed from 5cm)
            else
                obj.gridSize = gridSize;
            end
        end
        
        function obj = loadPointCloud(obj, points)
            % Load point cloud data
            % points: Nx3 matrix of [x,y,z] coordinates
            obj.pointCloud = points;
        end
        
        function obj = processPointCloud(obj)
            % Main processing pipeline
            obj = obj.createHeightMap();
            obj = obj.computeNormals();
            obj = obj.assessTraversability();
        end
        
        function obj = createHeightMap(obj)
            % Create a height map from point cloud
            points = obj.pointCloud;
            
            % Define grid boundaries
            xMin = min(points(:,1)); xMax = max(points(:,1));
            yMin = min(points(:,2)); yMax = max(points(:,2));
            
            % Create grid
            [X, Y] = meshgrid(xMin:obj.gridSize:xMax, yMin:obj.gridSize:yMax);
            Z = zeros(size(X));
            
            % For each grid cell, find maximum height
            for i = 1:size(X,1)
                for j = 1:size(X,2)
                    % Find points in current cell
                    idx = find(points(:,1) >= X(i,j) & points(:,1) < X(i,j)+obj.gridSize & ...
                             points(:,2) >= Y(i,j) & points(:,2) < Y(i,j)+obj.gridSize);
                    
                    if ~isempty(idx)
                        Z(i,j) = max(points(idx,3));
                    else
                        Z(i,j) = NaN;
                    end
                end
            end
            
            obj.heightMap = struct('X', X, 'Y', Y, 'Z', Z);
        end
        
        function obj = computeNormals(obj)
            % Compute surface normals
            [Zx, Zy] = gradient(obj.heightMap.Z, obj.gridSize);
            
            % Normal vectors for each cell
            normals = zeros(size(obj.heightMap.Z,1), size(obj.heightMap.Z,2), 3);
            for i = 1:size(Zx,1)
                for j = 1:size(Zx,2)
                    normal = [-Zx(i,j), -Zy(i,j), 1];
                    normals(i,j,:) = normal / norm(normal);
                end
            end
            
            obj.normalMap = normals;
        end
        
        function obj = assessTraversability(obj)
            % Assess terrain traversability based on slope and roughness
            % Initialize traversability map (0: not traversable, 1: traversable)
            traversability = ones(size(obj.heightMap.Z));
            
            % Parameters
            maxSlope = 45; % Maximum traversable slope in degrees
            maxStepHeight = 0.2; % Maximum step height in meters
            
            % Check slope
            [Zx, Zy] = gradient(obj.heightMap.Z, obj.gridSize);
            slopes = atan2(sqrt(Zx.^2 + Zy.^2), 1) * 180/pi;
            traversability(slopes > maxSlope) = 0;
            
            % Check step heights
            for i = 2:size(obj.heightMap.Z,1)
                for j = 2:size(obj.heightMap.Z,2)
                    % Check height difference with neighbors
                    heightDiffs = abs([
                        obj.heightMap.Z(i,j) - obj.heightMap.Z(i-1,j);
                        obj.heightMap.Z(i,j) - obj.heightMap.Z(i,j-1)
                    ]);
                    
                    if any(heightDiffs > maxStepHeight)
                        traversability(i,j) = 0;
                    end
                end
            end
            
            obj.traversabilityMap = traversability;
        end
        
        function visualizeTerrain(obj)
            % Visualize processed terrain data
            figure('Name', 'Terrain Analysis');
            
            % Height map
            subplot(2,2,1)
            surf(obj.heightMap.X, obj.heightMap.Y, obj.heightMap.Z);
            title('Height Map');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            colorbar;
            
            % Surface normals
            subplot(2,2,2)
            quiver3(obj.heightMap.X, obj.heightMap.Y, obj.heightMap.Z, ...
                   obj.normalMap(:,:,1), obj.normalMap(:,:,2), obj.normalMap(:,:,3));
            title('Surface Normals');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            
            % Traversability map
            subplot(2,2,3)
            imagesc(obj.traversabilityMap);
            title('Traversability Map');
            colorbar;
            xlabel('X grid'); ylabel('Y grid');
            
            % 3D point cloud
            subplot(2,2,4)
            scatter3(obj.pointCloud(:,1), obj.pointCloud(:,2), obj.pointCloud(:,3), '.', 'MarkerEdgeAlpha', 0.5);
            title('Original Point Cloud');
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        end
    end
end 