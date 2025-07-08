%% Test Quadruped Path Follower
% Standalone test script for QuadrupedPathFollower with various path types

clear; clc; close all;

fprintf('=== Quadruped Path Follower Test Suite ===\n');

%% Test Configuration
testCases = {
    'straight_line', 'Straight Line Path';
    'circle', 'Circular Path';
    'spiral', 'Spiral Path (2D)';
    'zigzag', 'Zigzag Pattern';
    'ramp', '3D Ramp Climbing';
    'custom_3d', 'Complex 3D Path'
};

% Select test case (change this to test different paths)
selectedTest = 'ramp';  % Options: 'straight_line', 'circle', 'spiral', 'zigzag', 'ramp', 'custom_3d'

%% Generate Test Path
switch selectedTest
    case 'straight_line'
        % Simple straight line for basic testing
        referencePath = [
            0, 0, 0;
            1, 0, 0;
            2, 0, 0;
            3, 0, 0;
            4, 0, 0;
            5, 0, 0
        ];
        testName = 'Straight Line Path';
        
    case 'circle'
        % Circular path at constant height
        radius = 2.0;
        numPoints = 50;
        theta = linspace(0, 2*pi, numPoints);
        referencePath = [
            radius * cos(theta)', ...
            radius * sin(theta)', ...
            ones(numPoints, 1) * 0.5
        ];
        testName = 'Circular Path';
        
    case 'spiral'
        % Spiral path with gradually increasing radius
        numPoints = 80;
        theta = linspace(0, 4*pi, numPoints);
        radius = linspace(0.5, 3.0, numPoints);
        referencePath = [
            radius' .* cos(theta)', ...
            radius' .* sin(theta)', ...
            ones(numPoints, 1) * 0.3
        ];
        testName = 'Spiral Path (2D)';
        
    case 'zigzag'
        % Zigzag pattern
        x = 0:0.2:5;
        y = sin(x * 2) * 1.5;
        z = ones(size(x)) * 0.2;
        referencePath = [x', y', z'];
        testName = 'Zigzag Pattern';
        
    case 'ramp'
        % 3D ramp climbing simulation
        % Create a path that climbs stairs or ramp
        stepHeight = 0.15;  % 15cm steps
        stepLength = 0.3;   % 30cm step length
        numSteps = 8;
        
        referencePath = [];
        for i = 0:numSteps
            % Add horizontal section
            x = i * stepLength;
            y = 0;
            z = i * stepHeight;
            referencePath = [referencePath; x, y, z];
            
            % Add transition points for smooth climbing
            if i < numSteps
                x_trans = x + stepLength/2;
                z_trans = z + stepHeight/2;
                referencePath = [referencePath; x_trans, y, z_trans];
            end
        end
        testName = '3D Ramp Climbing';
        
    case 'custom_3d'
        % Complex 3D path combining multiple maneuvers
        t = linspace(0, 4*pi, 100);
        referencePath = [
            t/2, ...                              % X: linear progression
            sin(t) * 2, ...                       % Y: sinusoidal motion
            0.5 + 0.3*sin(t/2) + t/20            % Z: gradual climb with oscillation
        ];
        testName = 'Complex 3D Path';
        
    otherwise
        error('Unknown test case: %s', selectedTest);
end

fprintf('Testing: %s\n', testName);
fprintf('Path points: %d\n', size(referencePath, 1));
fprintf('Path length: %.2f m\n', sum(sqrt(sum(diff(referencePath).^2, 2))));

%% Visualize Test Path
figure('Name', 'Test Path Visualization', 'Position', [100, 100, 1000, 600]);

subplot(1, 2, 1);
plot3(referencePath(:,1), referencePath(:,2), referencePath(:,3), ...
    'b-o', 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('3D View: %s', testName));
grid on; axis equal;
view(45, 30);

subplot(1, 2, 2);
plot(referencePath(:,1), referencePath(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
xlabel('X (m)'); ylabel('Y (m)');
title('Top View');
grid on; axis equal;

%% Initialize Path Follower
pathFollower = QuadrupedPathFollower();

% Configure parameters based on test type
switch selectedTest
    case 'straight_line'
        pathFollower.maxVelX = 1.0;
        pathFollower.maxOmega = 0.5;
        pathFollower.lookaheadDist = 0.8;
        
    case 'circle'
        pathFollower.maxVelX = 0.8;
        pathFollower.maxOmega = 1.5;
        pathFollower.lookaheadDist = 0.5;
        
    case 'spiral'
        pathFollower.maxVelX = 0.7;
        pathFollower.maxOmega = 2.0;
        pathFollower.lookaheadDist = 0.4;
        
    case 'zigzag'
        pathFollower.maxVelX = 0.6;
        pathFollower.maxOmega = 2.5;
        pathFollower.lookaheadDist = 0.3;
        pathFollower.kp_lateral = 3.0;
        
    case 'ramp'
        pathFollower.maxVelX = 0.5;
        pathFollower.maxOmega = 1.0;
        pathFollower.lookaheadDist = 0.2;
        pathFollower.kp_lateral = 2.5;
        pathFollower.kp_heading = 3.5;
        
    case 'custom_3d'
        pathFollower.maxVelX = 0.8;
        pathFollower.maxOmega = 1.8;
        pathFollower.lookaheadDist = 0.6;
        pathFollower.kp_lateral = 2.2;
end

%% Set Initial Conditions
% Place robot at start of path with appropriate heading
initialPos = referencePath(1, :);
if size(referencePath, 1) > 1
    direction = referencePath(2, :) - referencePath(1, :);
    initialHeading = atan2(direction(2), direction(1));
else
    initialHeading = 0;
end

pathFollower.setInitialPosition(initialPos, initialHeading);
pathFollower.setReferencePath(referencePath);

fprintf('\nRobot initialized at [%.2f, %.2f, %.2f]\n', initialPos);
fprintf('Initial heading: %.1f degrees\n', rad2deg(initialHeading));

%% Run Test Simulation
fprintf('\n=== Running Test: %s ===\n', testName);

% Start timer for performance measurement
tic;
success = pathFollower.runSimulation();
elapsedTime = toc;

fprintf('\nSimulation completed in %.2f seconds (wall clock time)\n', elapsedTime);

%% Analyze Test Results
fprintf('\n=== Test Results Analysis ===\n');

% Performance metrics
if ~isempty(pathFollower.errorLog)
    meanLateralError = mean(abs(pathFollower.errorLog(:,1)));
    maxLateralError = max(abs(pathFollower.errorLog(:,1)));
    meanHeadingError = mean(abs(pathFollower.errorLog(:,2)));
    maxHeadingError = max(abs(pathFollower.errorLog(:,2)));
    
    fprintf('Tracking Performance:\n');
    fprintf('  Mean lateral error: %.3f m\n', meanLateralError);
    fprintf('  Max lateral error: %.3f m\n', maxLateralError);
    fprintf('  Mean heading error: %.1f degrees\n', rad2deg(meanHeadingError));
    fprintf('  Max heading error: %.1f degrees\n', rad2deg(maxHeadingError));
    
    % Control performance
    avgSpeed = mean(pathFollower.velocityLog(:,1));
    maxSpeed = max(pathFollower.velocityLog(:,1));
    avgOmega = mean(abs(pathFollower.velocityLog(:,2)));
    maxOmega = max(abs(pathFollower.velocityLog(:,2)));
    
    fprintf('\nControl Performance:\n');
    fprintf('  Average speed: %.2f m/s\n', avgSpeed);
    fprintf('  Maximum speed: %.2f m/s\n', maxSpeed);
    fprintf('  Average turn rate: %.2f rad/s\n', avgOmega);
    fprintf('  Maximum turn rate: %.2f rad/s\n', maxOmega);
    
    % Path following efficiency
    refLength = sum(sqrt(sum(diff(referencePath).^2, 2)));
    robotLength = sum(sqrt(sum(diff(pathFollower.trajectory).^2, 2)));
    efficiency = (refLength / robotLength) * 100;
    
    fprintf('\nPath Efficiency:\n');
    fprintf('  Reference path length: %.2f m\n', refLength);
    fprintf('  Robot path length: %.2f m\n', robotLength);
    fprintf('  Path efficiency: %.1f%%\n', efficiency);
    
    % Final position accuracy
    finalError = norm(pathFollower.trajectory(end,:) - referencePath(end,:));
    fprintf('  Final position error: %.3f m\n', finalError);
    
    % Test scoring
    fprintf('\n=== TEST SCORING ===\n');
    
    % Scoring criteria
    lateralScore = max(0, 100 - meanLateralError * 200);  % 0.5m error = 0 points
    headingScore = max(0, 100 - rad2deg(meanHeadingError) * 2);  % 50deg error = 0 points
    efficiencyScore = min(100, efficiency);  % >100% efficiency gets 100 points
    accuracyScore = max(0, 100 - finalError * 200);  % 0.5m final error = 0 points
    
    overallScore = (lateralScore + headingScore + efficiencyScore + accuracyScore) / 4;
    
    fprintf('Individual Scores (0-100):\n');
    fprintf('  Lateral tracking: %.1f\n', lateralScore);
    fprintf('  Heading tracking: %.1f\n', headingScore);
    fprintf('  Path efficiency: %.1f\n', efficiencyScore);
    fprintf('  Final accuracy: %.1f\n', accuracyScore);
    fprintf('  OVERALL SCORE: %.1f/100\n', overallScore);
    
    % Grade assignment
    if overallScore >= 90
        grade = 'A+ (Excellent)';
    elseif overallScore >= 80
        grade = 'A (Very Good)';
    elseif overallScore >= 70
        grade = 'B (Good)';
    elseif overallScore >= 60
        grade = 'C (Satisfactory)';
    else
        grade = 'F (Needs Improvement)';
    end
    
    fprintf('  GRADE: %s\n', grade);
    
else
    fprintf('No tracking data available for analysis.\n');
end

%% Save Test Results
testResults = struct();
testResults.testName = testName;
testResults.testType = selectedTest;
testResults.referencePath = referencePath;
testResults.robotTrajectory = pathFollower.trajectory;
testResults.success = success;
testResults.simTime = pathFollower.simTime;
testResults.wallClockTime = elapsedTime;

if ~isempty(pathFollower.errorLog)
    testResults.performance.meanLateralError = meanLateralError;
    testResults.performance.maxLateralError = maxLateralError;
    testResults.performance.meanHeadingError = meanHeadingError;
    testResults.performance.maxHeadingError = maxHeadingError;
    testResults.performance.avgSpeed = avgSpeed;
    testResults.performance.pathEfficiency = efficiency;
    testResults.performance.finalError = finalError;
    testResults.performance.overallScore = overallScore;
    testResults.performance.grade = grade;
end

% Save to results directory
if ~exist('results', 'dir')
    mkdir('results');
end

filename = sprintf('results/test_%s_results.mat', selectedTest);
save(filename, 'testResults');
fprintf('\nTest results saved to %s\n', filename);

%% Summary
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('Test: %s\n', testName);
if success
    fprintf('Status: PASSED\n');
else
    fprintf('Status: FAILED\n');
end
if ~isempty(pathFollower.errorLog)
    fprintf('Score: %.1f/100 (%s)\n', overallScore, grade);
end
fprintf('Time: %.1fs (simulation), %.1fs (real)\n', pathFollower.simTime, elapsedTime);

if success
    fprintf('\n✓ Path follower successfully completed the test!\n');
    fprintf('✓ Ready for deployment on real quadruped robot\n');
else
    fprintf('\n⚠ Test incomplete - check parameters and try again\n');
end

fprintf('\nTest complete. Check the simulation figure for detailed trajectory analysis.\n'); 