classdef QuadrupedPathFollower < handle
    properties
        % Robot state
        position        % Current position [x, y, z]
        orientation     % Current orientation [roll, pitch, yaw]
        velocity        % Current velocity [vx, vy, vz]
        angularVel      % Current angular velocity [wx, wy, wz]
        
        % Path following parameters
        referencePath   % Reference trajectory from TomogramProcessor
        currentTarget   % Current target point index
        lookaheadDist   % Lookahead distance for path following
        maxVelX         % Maximum forward velocity
        maxOmega        % Maximum angular velocity
        
        % Control parameters
        kp_lateral      % Proportional gain for lateral error
        kp_heading      % Proportional gain for heading error
        kd_lateral      % Derivative gain for lateral error
        kd_heading      % Derivative gain for heading error
        
        % Simulation parameters
        dt              % Time step
        simTime         % Current simulation time
        maxSimTime      % Maximum simulation time
        
        % Logging
        trajectory      % Logged trajectory
        velocityLog     % Logged velocity commands
        errorLog        % Logged tracking errors
        timeLog         % Logged time stamps
        
        % Terrain adaptation
        terrainBuffer   % Buffer for terrain-aware planning
        adaptiveGains   % Adaptive control gains based on terrain
        rlNetworkMode   % RL network adaptation mode
        
        % Low-pass filter and predictive control
        lpf_cutoff_freq    % Low-pass filter cutoff frequency [Hz]
        lpf_alpha_vel      % Filter coefficient for velocity (0-1)
        lpf_alpha_omega    % Filter coefficient for angular velocity (0-1)
        filtered_velX      % Filtered forward velocity
        filtered_omega     % Filtered angular velocity
        prediction_horizon % Number of future waypoints to consider
        prediction_weight  % Weight for predictive component (0-1)
        max_accel_x       % Maximum forward acceleration [m/s²]
        max_accel_omega   % Maximum angular acceleration [rad/s²]
        prev_velX         % Previous velocity command
        prev_omega        % Previous angular velocity command
    end
    
    methods
        function obj = QuadrupedPathFollower()
            % Constructor
            obj.initializeParameters();
            obj.initializeState();
            obj.initializeLogging();
        end
        
        function initializeParameters(obj)
            % Initialize control and simulation parameters
            
            % Path following parameters
            obj.lookaheadDist = 0.5;    % 50cm lookahead distance
            obj.maxVelX = 0.8;          % 0.8 m/s max forward velocity
            obj.maxOmega = 1.2;         % 1.2 rad/s max angular velocity
            
            % Control gains (tuned for quadruped)
            obj.kp_lateral = 2.0;       % Lateral error gain
            obj.kp_heading = 3.0;       % Heading error gain
            obj.kd_lateral = 0.5;       % Lateral derivative gain
            obj.kd_heading = 0.8;       % Heading derivative gain
            
            % Simulation parameters
            obj.dt = 0.05;              % 50ms time step (20Hz control)
            obj.maxSimTime = 60.0;      % 60 seconds max simulation
            
            % Terrain adaptation parameters
            obj.terrainBuffer = [];
            obj.adaptiveGains = struct('lateral_mult', 1.0, 'heading_mult', 1.0);
            obj.rlNetworkMode = 'adaptive';  % 'adaptive' or 'conservative'
            
            % Low-pass filter parameters
            obj.lpf_cutoff_freq = 2.0;      % 2Hz cutoff frequency
            obj.lpf_alpha_vel = 0.8;        % Filter coefficient for velocity (higher=more filtering)
            obj.lpf_alpha_omega = 0.7;      % Filter coefficient for angular velocity
            
            % Predictive control parameters
            obj.prediction_horizon = 5;     % Look ahead 5 waypoints
            obj.prediction_weight = 0.3;    % 30% predictive, 70% current control
            
            % Acceleration limits (for smooth control)
            obj.max_accel_x = 2.0;          % 2 m/s² max forward acceleration
            obj.max_accel_omega = 4.0;      % 4 rad/s² max angular acceleration
        end
        
        function initializeState(obj)
            % Initialize robot state
            obj.position = [0; 0; 0];
            obj.orientation = [0; 0; 0];  % [roll, pitch, yaw]
            obj.velocity = [0; 0; 0];
            obj.angularVel = [0; 0; 0];
            obj.currentTarget = 1;
            obj.simTime = 0;
            
            % Initialize filter states
            obj.filtered_velX = 0;
            obj.filtered_omega = 0;
            obj.prev_velX = 0;
            obj.prev_omega = 0;
        end
        
        function initializeLogging(obj)
            % Initialize logging arrays
            obj.trajectory = [];
            obj.velocityLog = [];
            obj.errorLog = [];
            obj.timeLog = [];
        end
        
        function setReferencePath(obj, path)
            % Set the reference path from TomogramProcessor
            % path: Nx3 matrix [x, y, z] waypoints
            
            if size(path, 2) ~= 3
                error('Path must be Nx3 matrix [x, y, z]');
            end
            
            obj.referencePath = path;
            obj.currentTarget = 1;
            
            fprintf('Path follower: Loaded path with %d waypoints\n', size(path, 1));
            fprintf('Path length: %.2f m\n', obj.calculatePathLength(path));
            fprintf('Height change: %.2f m\n', path(end,3) - path(1,3));
        end
        
        function setInitialPosition(obj, pos, yaw)
            % Set initial robot position and orientation
            % pos: [x, y, z] initial position
            % yaw: initial heading angle (radians)
            
            if nargin < 3
                yaw = 0;
            end
            
            obj.position = pos(:);
            obj.orientation(3) = yaw;  % Set yaw
            
            fprintf('Robot initialized at [%.2f, %.2f, %.2f], yaw=%.2f°\n', ...
                pos(1), pos(2), pos(3), rad2deg(yaw));
        end
        
        function [velX, omega] = computeControlCommand(obj)
            % Compute velocity commands using Enhanced Pure Pursuit with 
            % Low-Pass Filtering and Predictive Control
            % Returns: velX (forward velocity), omega (angular velocity)
            
            if isempty(obj.referencePath)
                velX = 0; omega = 0;
                return;
            end
            
            % Find target point using lookahead distance
            targetPoint = obj.findLookaheadTarget();
            
            if isempty(targetPoint)
                % End of path reached - apply smooth deceleration
                [velX, omega] = obj.applyLowPassFilter(0, 0);
                return;
            end
            
            % === STEP 1: Current Control (Pure Pursuit) ===
            % Calculate control errors
            [lateralError, headingError, curvature] = obj.calculateTrackingErrors(targetPoint);
            
            % Terrain-adaptive gain adjustment
            [adaptiveLateralGain, adaptiveHeadingGain] = obj.getAdaptiveGains();
            
            % Pure Pursuit control law with 3D adaptation
            % Forward velocity control based on path curvature and terrain
            curvatureFactor = 1.0 / (1.0 + abs(curvature) * 2.0);
            terrainFactor = obj.getTerrainSpeedFactor();
            current_velX = obj.maxVelX * curvatureFactor * terrainFactor;
            
            % Angular velocity control for path following
            lateralControl = obj.kp_lateral * adaptiveLateralGain * lateralError;
            headingControl = obj.kp_heading * adaptiveHeadingGain * headingError;
            
            % Add derivative terms for smoothness
            persistent prevLateralError prevHeadingError
            if isempty(prevLateralError)
                prevLateralError = 0;
                prevHeadingError = 0;
            end
            
            lateralDerivative = (lateralError - prevLateralError) / obj.dt;
            headingDerivative = (headingError - prevHeadingError) / obj.dt;
            
            current_omega = lateralControl + headingControl + ...
                           obj.kd_lateral * lateralDerivative + ...
                           obj.kd_heading * headingDerivative;
            
            % Apply velocity limits
            current_velX = max(0, min(current_velX, obj.maxVelX));
            current_omega = max(-obj.maxOmega, min(current_omega, obj.maxOmega));
            
            % === STEP 2: Predictive Control ===
            [pred_velX, pred_omega] = obj.computePredictiveControl();
            
            % === STEP 3: Combine Current and Predictive Control ===
            % Weighted combination of current and predictive control
            combined_velX = (1 - obj.prediction_weight) * current_velX + ...
                           obj.prediction_weight * pred_velX;
            combined_omega = (1 - obj.prediction_weight) * current_omega + ...
                            obj.prediction_weight * pred_omega;
            
            % Apply velocity limits again
            combined_velX = max(0, min(combined_velX, obj.maxVelX));
            combined_omega = max(-obj.maxOmega, min(combined_omega, obj.maxOmega));
            
            % RL network mode adjustments
            if strcmp(obj.rlNetworkMode, 'conservative')
                combined_velX = combined_velX * 0.7;  % Reduce speed for conservative mode
            end
            
            % === STEP 4: Apply Low-Pass Filter ===
            [velX, omega] = obj.applyLowPassFilter(combined_velX, combined_omega);
            
            % Update previous errors
            prevLateralError = lateralError;
            prevHeadingError = headingError;
            
            % Log control data (using filtered values)
            obj.logControlData(velX, omega, lateralError, headingError, targetPoint);
        end
        
        function targetPoint = findLookaheadTarget(obj)
            % Find the target point at lookahead distance along the path (2D)
            % Only consider XY distance for realistic quadruped navigation
            
            if obj.currentTarget > size(obj.referencePath, 1)
                targetPoint = [];
                return;
            end
            
            currentPos = obj.position;
            
            % Start from current target and find point at lookahead distance (XY only)
            for i = obj.currentTarget:size(obj.referencePath, 1)
                % Only consider XY distance - height is handled by RL
                dist2D = norm(obj.referencePath(i, 1:2) - currentPos(1:2)');
                
                if dist2D >= obj.lookaheadDist
                    targetPoint = obj.referencePath(i, :);
                    obj.currentTarget = i;
                    return;
                end
            end
            
            % If no point found at exact lookahead distance, use the last point
            targetPoint = obj.referencePath(end, :);
            obj.currentTarget = size(obj.referencePath, 1);
        end
        
        function [lateralError, headingError, curvature] = calculateTrackingErrors(obj, targetPoint)
            % Calculate lateral and heading errors for 2D path following
            % Only consider XY plane - Z is handled by RL network
            
            currentPos = obj.position;
            currentYaw = obj.orientation(3);
            
            % Vector from robot to target (XY plane only)
            targetVector2D = targetPoint(1:2) - currentPos(1:2)';
            targetDistance = norm(targetVector2D);
            
            if targetDistance < 0.01
                lateralError = 0;
                headingError = 0;
                curvature = 0;
                return;
            end
            
            % Desired heading (in global frame, XY plane only)
            desiredYaw = atan2(targetVector2D(2), targetVector2D(1));
            
            % Heading error (wrapped to [-pi, pi])
            headingError = obj.wrapAngle(desiredYaw - currentYaw);
            
            % Lateral error (perpendicular distance to desired path in XY plane)
            % Project target vector onto robot's left side
            robotLeftVector = [-sin(currentYaw), cos(currentYaw)];
            lateralError = dot(targetVector2D, robotLeftVector);
            
            % Estimate path curvature using current and next waypoints (2D)
            curvature = obj.estimatePathCurvature();
        end
        
        function curvature = estimatePathCurvature(obj)
            % Estimate local path curvature for speed adjustment
            
            if obj.currentTarget < 2 || obj.currentTarget >= size(obj.referencePath, 1)
                curvature = 0;
                return;
            end
            
            % Use three consecutive points to estimate curvature
            p1 = obj.referencePath(obj.currentTarget-1, 1:2);
            p2 = obj.referencePath(obj.currentTarget, 1:2);
            p3 = obj.referencePath(obj.currentTarget+1, 1:2);
            
            % Calculate curvature using circumcircle method
            a = norm(p2 - p1);
            b = norm(p3 - p2);
            c = norm(p3 - p1);
            
            if a < 0.01 || b < 0.01 || c < 0.01
                curvature = 0;
                return;
            end
            
            % Area of triangle
            s = (a + b + c) / 2;
            area = sqrt(max(0, s * (s-a) * (s-b) * (s-c)));
            
            if area < 0.001
                curvature = 0;
            else
                curvature = 4 * area / (a * b * c);
            end
        end
        
        function [lateralGain, headingGain] = getAdaptiveGains(obj)
            % Get adaptive control gains based on terrain and robot state
            
            % Base gains
            lateralGain = obj.adaptiveGains.lateral_mult;
            headingGain = obj.adaptiveGains.heading_mult;
            
            % Adjust gains based on height (stairs, slopes)
            currentHeight = obj.position(3);
            if obj.currentTarget <= size(obj.referencePath, 1)
                targetHeight = obj.referencePath(obj.currentTarget, 3);
                heightDiff = abs(targetHeight - currentHeight);
                
                % Increase lateral gain for height transitions
                if heightDiff > 0.05  % 5cm height difference
                    lateralGain = lateralGain * (1.0 + heightDiff);
                    headingGain = headingGain * (1.0 + heightDiff * 0.5);
                end
            end
            
            % Adjust based on current velocity (higher speed = lower gains)
            speedFactor = norm(obj.velocity) / obj.maxVelX;
            lateralGain = lateralGain * (1.0 - speedFactor * 0.3);
            headingGain = headingGain * (1.0 - speedFactor * 0.2);
            
            % Keep gains within reasonable bounds
            lateralGain = max(0.1, min(3.0, lateralGain));
            headingGain = max(0.1, min(3.0, headingGain));
        end
        
        function speedFactor = getTerrainSpeedFactor(obj)
            % Get speed reduction factor based on terrain complexity
            
            speedFactor = 1.0;  % Default full speed
            
            if obj.currentTarget <= size(obj.referencePath, 1)
                % Check height gradient
                if obj.currentTarget > 1
                    heightChange = abs(obj.referencePath(obj.currentTarget, 3) - ...
                                     obj.referencePath(obj.currentTarget-1, 3));
                    
                    % Reduce speed for steep terrain
                    if heightChange > 0.1  % 10cm height change
                        speedFactor = speedFactor * 0.6;
                    elseif heightChange > 0.05  % 5cm height change
                        speedFactor = speedFactor * 0.8;
                    end
                end
                
                % Check path curvature
                curvature = obj.estimatePathCurvature();
                if curvature > 1.0  % High curvature
                    speedFactor = speedFactor * 0.7;
                elseif curvature > 0.5  % Medium curvature
                    speedFactor = speedFactor * 0.85;
                end
            end
            
            % Ensure minimum speed for progress
            speedFactor = max(0.3, speedFactor);
        end
        
        function [pred_velX, pred_omega] = computePredictiveControl(obj)
            % Compute predictive control commands based on future waypoints
            % This helps reduce jerky movements by anticipating path changes
            
            if isempty(obj.referencePath) || obj.currentTarget > size(obj.referencePath, 1)
                pred_velX = 0;
                pred_omega = 0;
                return;
            end
            
            % Look ahead at future waypoints
            horizon = min(obj.prediction_horizon, size(obj.referencePath, 1) - obj.currentTarget + 1);
            
            if horizon < 2
                pred_velX = 0;
                pred_omega = 0;
                return;
            end
            
            % Calculate predicted errors for future waypoints
            total_lateral_error = 0;
            total_heading_error = 0;
            total_curvature = 0;
            
            for i = 1:horizon
                future_idx = obj.currentTarget + i - 1;
                if future_idx > size(obj.referencePath, 1)
                    break;
                end
                
                future_target = obj.referencePath(future_idx, :);
                
                % Calculate future errors (simplified)
                currentPos = obj.position;
                currentYaw = obj.orientation(3);
                
                % Vector from robot to future target (XY plane only)
                targetVector2D = future_target(1:2) - currentPos(1:2)';
                targetDistance = norm(targetVector2D);
                
                if targetDistance > 0.01
                    % Future heading
                    desiredYaw = atan2(targetVector2D(2), targetVector2D(1));
                    headingError = obj.wrapAngle(desiredYaw - currentYaw);
                    
                    % Future lateral error
                    robotLeftVector = [-sin(currentYaw), cos(currentYaw)];
                    lateralError = dot(targetVector2D, robotLeftVector);
                    
                    % Weight errors by distance (closer = more important)
                    weight = 1.0 / (i * 1.5);  % Exponential decay
                    
                    total_lateral_error = total_lateral_error + lateralError * weight;
                    total_heading_error = total_heading_error + headingError * weight;
                    
                    % Estimate future curvature
                    if future_idx > 1 && future_idx < size(obj.referencePath, 1)
                        p1 = obj.referencePath(future_idx-1, 1:2);
                        p2 = obj.referencePath(future_idx, 1:2);
                        p3 = obj.referencePath(future_idx+1, 1:2);
                        
                        % Simple curvature estimation
                        v1 = p2 - p1;
                        v2 = p3 - p2;
                        if norm(v1) > 0.01 && norm(v2) > 0.01
                            angle_change = abs(atan2(v2(2), v2(1)) - atan2(v1(2), v1(1)));
                            if angle_change > pi
                                angle_change = 2*pi - angle_change;
                            end
                            curvature = angle_change / (norm(v1) + norm(v2));
                            total_curvature = total_curvature + curvature * weight;
                        end
                    end
                end
            end
            
            % Compute predictive control commands
            [adaptiveLateralGain, adaptiveHeadingGain] = obj.getAdaptiveGains();
            
            % Predictive velocity control
            curvatureFactor = 1.0 / (1.0 + total_curvature * 3.0);
            terrainFactor = obj.getTerrainSpeedFactor();
            pred_velX = obj.maxVelX * curvatureFactor * terrainFactor;
            
            % Predictive angular velocity control
            lateral_pred_control = obj.kp_lateral * adaptiveLateralGain * total_lateral_error;
            heading_pred_control = obj.kp_heading * adaptiveHeadingGain * total_heading_error;
            
            pred_omega = lateral_pred_control + heading_pred_control;
            
            % Apply limits
            pred_velX = max(0, min(pred_velX, obj.maxVelX));
            pred_omega = max(-obj.maxOmega, min(pred_omega, obj.maxOmega));
        end
        
        function [smooth_velX, smooth_omega] = applyLowPassFilter(obj, raw_velX, raw_omega)
            % Apply low-pass filter to smooth control commands
            % This reduces high-frequency noise and jerky movements
            
            % Low-pass filter: y[n] = α * y[n-1] + (1-α) * x[n]
            % where α is the filter coefficient (0 < α < 1)
            
            % Apply exponential moving average filter
            obj.filtered_velX = obj.lpf_alpha_vel * obj.filtered_velX + ...
                               (1 - obj.lpf_alpha_vel) * raw_velX;
            
            obj.filtered_omega = obj.lpf_alpha_omega * obj.filtered_omega + ...
                                (1 - obj.lpf_alpha_omega) * raw_omega;
            
            % Apply acceleration limits for additional smoothness
            dt = obj.dt;
            
            % Velocity acceleration limiting
            max_vel_change = obj.max_accel_x * dt;
            vel_change = obj.filtered_velX - obj.prev_velX;
            if abs(vel_change) > max_vel_change
                obj.filtered_velX = obj.prev_velX + sign(vel_change) * max_vel_change;
            end
            
            % Angular velocity acceleration limiting
            max_omega_change = obj.max_accel_omega * dt;
            omega_change = obj.filtered_omega - obj.prev_omega;
            if abs(omega_change) > max_omega_change
                obj.filtered_omega = obj.prev_omega + sign(omega_change) * max_omega_change;
            end
            
            % Update previous values
            obj.prev_velX = obj.filtered_velX;
            obj.prev_omega = obj.filtered_omega;
            
            % Return smoothed values
            smooth_velX = obj.filtered_velX;
            smooth_omega = obj.filtered_omega;
        end
        
        function updateRobotState(obj, velX, omega)
            % Update robot state using 2D kinematic model + RL terrain adaptation
            % This simulates the RL network's response to velocity commands
            
            currentYaw = obj.orientation(3);
            
            % Update XY position using 2D kinematic model (standard diff-drive)
            obj.position(1) = obj.position(1) + velX * cos(currentYaw) * obj.dt;
            obj.position(2) = obj.position(2) + velX * sin(currentYaw) * obj.dt;
            
            % Update height using RL-style terrain adaptation
            if ~isempty(obj.referencePath)
                % Simulate RL network terrain adaptation
                adaptiveHeight = obj.simulateRLTerrainAdaptation();
                obj.position(3) = adaptiveHeight;
            end
            
            % Update orientation (only yaw, roll/pitch handled by RL)
            obj.orientation(3) = obj.wrapAngle(obj.orientation(3) + omega * obj.dt);
            
            % Update velocity (with simple dynamics)
            obj.velocity(1) = velX * cos(currentYaw);
            obj.velocity(2) = velX * sin(currentYaw);
            obj.angularVel(3) = omega;
            
            % Update simulation time
            obj.simTime = obj.simTime + obj.dt;
        end
        
        function height = simulateRLTerrainAdaptation(obj)
            % Simulate RL network terrain adaptation behavior
            % The RL network naturally adapts to terrain based on local sensing
            
            if size(obj.referencePath, 1) < 2
                height = obj.position(3);
                return;
            end
            
            currentPos2D = obj.position(1:2);
            currentHeight = obj.position(3);
            
            % Find the expected terrain height at current XY position
            expectedHeight = obj.getTerrainHeightAtPosition(currentPos2D);
            
            % Simulate RL network adaptation characteristics:
            % 1. Gradual adaptation (not instant jumping)
            % 2. Predictive stepping (looking ahead for terrain changes)
            % 3. Natural foot placement optimization
            
            heightDifference = expectedHeight - currentHeight;
            
            % RL adaptation parameters
            maxClimbRate = 0.6;     % m/s - realistic climbing speed for quadruped
            maxDescendRate = 0.8;   % m/s - faster descending than climbing
            adaptationGain = 0.8;   % How quickly the RL adapts (0-1)
            
            % Determine adaptation rate based on terrain type
            if heightDifference > 0
                % Climbing - more conservative
                maxRate = maxClimbRate;
                gain = adaptationGain * 0.7;  % Slower for climbing
            else
                % Descending - more aggressive
                maxRate = maxDescendRate;
                gain = adaptationGain;
            end
            
            % Apply smooth adaptation
            desiredHeightChange = heightDifference * gain;
            maxChangeThisStep = maxRate * obj.dt;
            
            if abs(desiredHeightChange) > maxChangeThisStep
                heightChange = sign(desiredHeightChange) * maxChangeThisStep;
            else
                heightChange = desiredHeightChange;
            end
            
            % Add some natural height variation (foot placement optimization)
            naturalVariation = 0.01 * sin(obj.simTime * 8);  % 1cm variation at 8Hz
            
            height = currentHeight + heightChange + naturalVariation;
            
            % Ensure height stays within reasonable bounds
            minHeight = min(obj.referencePath(:,3)) - 0.5;
            maxHeight = max(obj.referencePath(:,3)) + 0.5;
            height = max(minHeight, min(maxHeight, height));
        end
        
        function terrainHeight = getTerrainHeightAtPosition(obj, xyPosition)
            % Get expected terrain height at given XY position
            % This simulates what the RL network "sees" from local sensors
            
            if size(obj.referencePath, 1) < 2
                terrainHeight = obj.position(3);
                return;
            end
            
            % Find nearest path points for interpolation
            distances = zeros(size(obj.referencePath, 1), 1);
            for i = 1:size(obj.referencePath, 1)
                distances(i) = norm(obj.referencePath(i, 1:2) - xyPosition');
            end
            
            [~, sortedIdx] = sort(distances);
            
            % Use weighted interpolation of nearby points
            numNearby = min(3, length(sortedIdx));
            weights = zeros(numNearby, 1);
            heights = zeros(numNearby, 1);
            
            for i = 1:numNearby
                idx = sortedIdx(i);
                dist = distances(idx);
                weights(i) = 1 / (dist + 0.01);  % Inverse distance weighting
                heights(i) = obj.referencePath(idx, 3);
            end
            
            % Normalize weights and compute weighted average
            weights = weights / sum(weights);
            terrainHeight = sum(weights .* heights);
            
            % Add some sensor noise/uncertainty
            sensorNoise = 0.005 * randn();  % 5mm sensor noise
            terrainHeight = terrainHeight + sensorNoise;
        end
        
        function logControlData(obj, velX, omega, lateralError, headingError, targetPoint)
            % Log control data for analysis
            
            obj.trajectory = [obj.trajectory; obj.position'];
            obj.velocityLog = [obj.velocityLog; velX, omega];
            obj.errorLog = [obj.errorLog; lateralError, headingError];
            obj.timeLog = [obj.timeLog; obj.simTime];
        end
        
        function isComplete = runSimulation(obj)
            % Run the complete path following simulation
            
            fprintf('\n=== Starting Path Following Simulation ===\n');
            fprintf('Path points: %d\n', size(obj.referencePath, 1));
            fprintf('Control frequency: %.1f Hz\n', 1/obj.dt);
            
            % Initialize visualization
            fig = figure('Name', 'Quadruped Path Following', 'Position', [100, 100, 1200, 800]);
            
            isComplete = false;
            frameCount = 0;
            
            while obj.simTime < obj.maxSimTime && ~obj.isPathComplete()
                % Compute control commands
                [velX, omega] = obj.computeControlCommand();
                
                % Update robot state
                obj.updateRobotState(velX, omega);
                
                % Visualize every 10 frames (2Hz visualization)
                frameCount = frameCount + 1;
                if mod(frameCount, 10) == 0
                    obj.visualizeSimulation(fig);
                    pause(0.01);  % Small pause for visualization
                end
                
                % Check for completion
                if obj.isPathComplete()
                    isComplete = true;
                    break;
                end
            end
            
            % Final visualization and analysis
            obj.visualizeSimulation(fig);
            obj.analyzePerformance();
            
            if isComplete
                fprintf('\nPath following completed successfully!\n');
            else
                fprintf('\nSimulation ended (time limit or other condition)\n');
            end
        end
        
        function complete = isPathComplete(obj)
            % Check if path following is complete (2D navigation)
            % Only consider XY distance - height naturally adapts via RL
            
            if isempty(obj.referencePath)
                complete = true;
                return;
            end
            
            % Check XY distance to final goal (ignore height difference)
            finalGoal = obj.referencePath(end, 1:2);
            distToGoal2D = norm(obj.position(1:2) - finalGoal');
            
            complete = (distToGoal2D < 0.2);  % 20cm XY tolerance
        end
        
        function visualizeSimulation(obj, fig)
            % Real-time visualization of path following
            
            figure(fig);
            clf;
            
            % 3D view
            subplot(2, 2, [1, 3]);
            hold on;
            
            % Plot reference path
            if ~isempty(obj.referencePath)
                plot3(obj.referencePath(:,1), obj.referencePath(:,2), obj.referencePath(:,3), ...
                    'b-', 'LineWidth', 2, 'DisplayName', 'Reference Path');
                
                % Mark current target
                if obj.currentTarget <= size(obj.referencePath, 1)
                    plot3(obj.referencePath(obj.currentTarget,1), ...
                          obj.referencePath(obj.currentTarget,2), ...
                          obj.referencePath(obj.currentTarget,3), ...
                          'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', ...
                          'DisplayName', 'Current Target');
                end
            end
            
            % Plot robot trajectory
            if size(obj.trajectory, 1) > 1
                plot3(obj.trajectory(:,1), obj.trajectory(:,2), obj.trajectory(:,3), ...
                    'g--', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');
            end
            
            % Plot current robot position and orientation
            obj.plotRobot3D();
            
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title(sprintf('3D Path Following (t=%.1fs)', obj.simTime));
            legend('Location', 'best');
            grid on; axis equal;
            view(45, 30);
            
            % Top view
            subplot(2, 2, 2);
            hold on;
            if ~isempty(obj.referencePath)
                plot(obj.referencePath(:,1), obj.referencePath(:,2), 'b-', 'LineWidth', 2);
            end
            if size(obj.trajectory, 1) > 1
                plot(obj.trajectory(:,1), obj.trajectory(:,2), 'g--', 'LineWidth', 1.5);
            end
            obj.plotRobot2D();
            xlabel('X (m)'); ylabel('Y (m)');
            title('Top View');
            grid on; axis equal;
            
            % Control signals (Quadruped RL Network Interface)
            subplot(2, 2, 4);
            if ~isempty(obj.timeLog)
                yyaxis left;
                plot(obj.timeLog, obj.velocityLog(:,1), 'b-', 'LineWidth', 1.5);
                ylabel('Forward Velocity (m/s)', 'Color', 'b');
                
                yyaxis right;
                plot(obj.timeLog, obj.velocityLog(:,2), 'r-', 'LineWidth', 1.5);
                ylabel('Angular Velocity (rad/s)', 'Color', 'r');
                
                xlabel('Time (s)');
                title('Quadruped Control Commands (2D Navigation)');
                grid on;
                
                % Add annotation
                text(0.02, 0.95, 'RL Network Interface:', 'Units', 'normalized', ...
                     'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
                text(0.02, 0.88, '• velocity_x: Forward speed', 'Units', 'normalized', ...
                     'FontSize', 7, 'Color', 'b');
                text(0.02, 0.82, '• omega: Turn rate', 'Units', 'normalized', ...
                     'FontSize', 7, 'Color', 'r');
                text(0.02, 0.76, '• Height/Gait: Auto-adapted', 'Units', 'normalized', ...
                     'FontSize', 7, 'Color', [0.5, 0.5, 0.5]);
            end
            
            drawnow;
        end
        
        function plotRobot3D(obj)
            % Plot robot in 3D view
            
            pos = obj.position;
            yaw = obj.orientation(3);
            
            % Robot body representation (rectangular footprint)
            length = 0.4;  % 40cm length
            width = 0.2;   % 20cm width
            
            % Robot corners in body frame
            corners_body = [
                length/2, -width/2, 0;
                length/2,  width/2, 0;
                -length/2, width/2, 0;
                -length/2, -width/2, 0;
                length/2, -width/2, 0  % Close the shape
            ];
            
            % Rotation matrix for yaw
            R = [cos(yaw), -sin(yaw), 0;
                 sin(yaw),  cos(yaw), 0;
                 0,         0,        1];
            
            % Transform to world frame
            corners_world = (R * corners_body')' + pos';
            
            % Plot robot body
            plot3(corners_world(:,1), corners_world(:,2), corners_world(:,3), ...
                'k-', 'LineWidth', 2);
            
            % Plot robot heading direction
            headingEnd = pos' + 0.3 * [cos(yaw), sin(yaw), 0];
            plot3([pos(1), headingEnd(1)], [pos(2), headingEnd(2)], [pos(3), headingEnd(3)], ...
                'r-', 'LineWidth', 3);
            
            % Mark robot center
            plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        end
        
        function plotRobot2D(obj)
            % Plot robot in 2D top view
            
            pos = obj.position;
            yaw = obj.orientation(3);
            
            % Robot body
            length = 0.4;
            width = 0.2;
            
            corners_body = [
                length/2, -width/2;
                length/2,  width/2;
                -length/2, width/2;
                -length/2, -width/2;
                length/2, -width/2
            ];
            
            R2D = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
            corners_world = (R2D * corners_body')' + pos(1:2)';
            
            plot(corners_world(:,1), corners_world(:,2), 'k-', 'LineWidth', 2);
            
            % Heading direction
            headingEnd = pos(1:2)' + 0.3 * [cos(yaw), sin(yaw)];
            plot([pos(1), headingEnd(1)], [pos(2), headingEnd(2)], 'r-', 'LineWidth', 3);
            
            plot(pos(1), pos(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        end
        
        function analyzePerformance(obj)
            % Analyze path following performance
            
            fprintf('\n=== Path Following Performance Analysis ===\n');
            
            if isempty(obj.errorLog)
                fprintf('No data to analyze.\n');
                return;
            end
            
            % Distance metrics
            totalDistance = obj.calculateTotalDistance();
            pathLength = obj.calculatePathLength(obj.referencePath);
            
            fprintf('Distance Metrics:\n');
            fprintf('  Reference path length: %.2f m\n', pathLength);
            fprintf('  Actual distance traveled: %.2f m\n', totalDistance);
            fprintf('  Distance efficiency: %.1f%%\n', (pathLength/totalDistance)*100);
            
            % Tracking errors
            lateralErrors = obj.errorLog(:, 1);
            headingErrors = obj.errorLog(:, 2);
            
            fprintf('\nTracking Errors:\n');
            fprintf('  Mean lateral error: %.3f m\n', mean(abs(lateralErrors)));
            fprintf('  Max lateral error: %.3f m\n', max(abs(lateralErrors)));
            fprintf('  RMS lateral error: %.3f m\n', rms(lateralErrors));
            
            fprintf('  Mean heading error: %.2f°\n', rad2deg(mean(abs(headingErrors))));
            fprintf('  Max heading error: %.2f°\n', rad2deg(max(abs(headingErrors))));
            fprintf('  RMS heading error: %.2f°\n', rad2deg(rms(headingErrors)));
            
            % Velocity statistics
            velocities = obj.velocityLog(:, 1);
            angularVels = obj.velocityLog(:, 2);
            
            fprintf('\nControl Statistics:\n');
            fprintf('  Mean forward velocity: %.2f m/s\n', mean(velocities));
            fprintf('  Max forward velocity: %.2f m/s\n', max(velocities));
            fprintf('  Mean angular velocity: %.2f rad/s\n', mean(abs(angularVels)));
            fprintf('  Max angular velocity: %.2f rad/s\n', max(abs(angularVels)));
            
            % Time performance
            fprintf('\nTime Performance:\n');
            fprintf('  Total simulation time: %.1f s\n', obj.simTime);
            fprintf('  Average speed: %.2f m/s\n', totalDistance / obj.simTime);
            
            % Final position error
            if ~isempty(obj.referencePath)
                finalError = norm(obj.position - obj.referencePath(end, :)');
                fprintf('  Final position error: %.3f m\n', finalError);
            end
        end
        
        function distance = calculateTotalDistance(obj)
            % Calculate total distance traveled by robot
            
            if size(obj.trajectory, 1) < 2
                distance = 0;
                return;
            end
            
            diffs = diff(obj.trajectory);
            distances = sqrt(sum(diffs.^2, 2));
            distance = sum(distances);
        end
        
        function length = calculatePathLength(obj, path)
            % Calculate length of a path
            
            if size(path, 1) < 2
                length = 0;
                return;
            end
            
            diffs = diff(path);
            distances = sqrt(sum(diffs.^2, 2));
            length = sum(distances);
        end
        
        function angle = wrapAngle(~, angle)
            % Wrap angle to [-pi, pi]
            angle = mod(angle + pi, 2*pi) - pi;
        end
        
        function saveResults(obj, filename)
            % Save simulation results
            
            if nargin < 2
                filename = 'path_following_results.mat';
            end
            
            results = struct();
            results.referencePath = obj.referencePath;
            results.trajectory = obj.trajectory;
            results.velocityLog = obj.velocityLog;
            results.errorLog = obj.errorLog;
            results.timeLog = obj.timeLog;
            results.parameters = struct();
            results.parameters.lookaheadDist = obj.lookaheadDist;
            results.parameters.maxVelX = obj.maxVelX;
            results.parameters.maxOmega = obj.maxOmega;
            results.parameters.dt = obj.dt;
            
            save(filename, 'results');
            fprintf('Results saved to %s\n', filename);
        end
    end
end 