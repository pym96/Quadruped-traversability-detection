% 1. Load or generate point cloud
pointCloud = rand(10000, 3) * 10; % Replace with pcread('your_file.pcd')

% 2. Create tomogram slices
gridSize = 0.5;
xGrid = 0:gridSize:10;
yGrid = 0:gridSize:10;
slices = cell(length(xGrid)-1, length(yGrid)-1);
for i = 1:length(xGrid)-1
    for j = 1:length(yGrid)-1
        idx = pointCloud(:,1) >= xGrid(i) & pointCloud(:,1) < xGrid(i+1) & ...
              pointCloud(:,2) >= yGrid(j) & pointCloud(:,2) < yGrid(j+1);
        zValues = pointCloud(idx, 3);
        slices{i,j} = [min(zValues), max(zValues)];
    end
end

% 3. Create cost map for path planning
map = zeros(length(xGrid)-1, length(yGrid)-1);
for i = 1:length(xGrid)-1
    for j = 1:length(yGrid)-1
        [ground, ceiling] = slices{i,j};
        if isempty(ground) || (ceiling - ground < 0.5)
            map(i,j) = 1; % Obstacle
        end
    end
end

% 4. Plan path using A*
planner = plannerAStar(binaryOccupancyMap(map));
start = [1, 1];
goal = [length(xGrid)-1, length(yGrid)-1];
path = plan(planner, start, goal);

% 5. Optimize trajectory (simplified for one segment)
t = 0:0.1:1;
beta = @(t) [1, t, t^2, t^3, t^4, t^5]';
p0 = [0, 0, 0];
pf = [1, 1, 0.5];
v0 = [0, 0, 0];
vf = [0, 0, 0];
costFun = @(sigma) sum((diff(diff(diff(sigma * beta(t)))).^2));
Aeq = [beta(0)', zeros(1,6); beta(1)', zeros(1,6); diff(beta(0))', zeros(1,6); diff(beta(1))', zeros(1,6)];
beq = [p0(1); pf(1); v0(1); vf(1)];
sigma_x = fmincon(costFun, rand(6,1), [], [], Aeq, beq);
trajectory_x = sigma_x' * beta(t);
% Repeat for y and z (or extend to 3D coefficients)

% 6. Adjust height for ceiling constraints
trajectory_z = zeros(size(trajectory_x));
for i = 1:length(path)
    grid_x = path(i,1); grid_y = path(i,2);
    ceiling = slices{grid_x, grid_y}(2);
    if trajectory_z(i) > ceiling - 0.5
        trajectory_z(i) = ceiling - 0.5;
    end
end

% 7. Visualize
figure;
pcshow(pointCloud);
hold on;
plot3(trajectory_x, trajectory_y, trajectory_z, 'r-', 'LineWidth', 2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Optimized 3D Trajectory');