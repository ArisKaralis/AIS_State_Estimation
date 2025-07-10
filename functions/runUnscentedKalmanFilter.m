function [estimates, stats] = runUnscentedKalmanFilter(data, q)
% RUNUNSCENTEDKALMANFILTER - Apply UKF to AIS data using trackingUKF
% Uses trackingUKF from Sensor Fusion and Tracking Toolbox
% Input:
%   data - AIS data table
%   q - Process noise intensity parameter (optional, default=0.1)

% Set default process noise if not provided
if nargin < 2
    q = 0.1;
end

fprintf('UKF: Using process noise intensity q = %.3f\n', q);

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

% Initialize state and covariance arrays
x_est = zeros(4, n);
P_est = zeros(4, 4, n);

% Use the measured position standard deviation from simulation
pos_std = 15;  % Approx 21m RMSE / sqrt(2)
vel_std = 1;   % From SOG RMSE of ~1 m/s

% Pre-process the trajectory to detect motion segments
[segments, segment_types] = detectMotionSegments(data);
fprintf('UKF: Detected %d motion segments in the data\n', length(segments));

% Two-stage filtering approach: first filter without COG to get better initial velocity
% STAGE 1: Run a simplified UKF on position-only data for initialization
ukf_init = trackingUKF(@cvStateTransitionFcn, @positionMeasurementFcn, ...
                      [data.x(1); 0; data.y(1); 0]);

% Configure initial UKF parameters for position-only filtering
ukf_init.StateCovariance = diag([pos_std^2, 100, pos_std^2, 100]);
ukf_init.MeasurementNoise = diag([pos_std^2, pos_std^2]);
ukf_init.ProcessNoise = eye(4) * 0.1;

% UKF-specific parameters (optimal for Gaussian distributions)
ukf_init.Alpha = 1e-3;  % Spread of sigma points around mean
ukf_init.Beta = 2;      % Prior knowledge parameter (optimal for Gaussian)
ukf_init.Kappa = 0;     % Secondary scaling parameter

% Run initialization filter for first 30 points
init_length = min(30, n);
init_states = zeros(4, init_length);

for k = 1:init_length
    if k > 1
        % Update process noise based on time step
        Q = [q*dt(k)^3/3, q*dt(k)^2/2, 0, 0;
             q*dt(k)^2/2, q*dt(k),    0, 0;
             0, 0, q*dt(k)^3/3, q*dt(k)^2/2;
             0, 0, q*dt(k)^2/2, q*dt(k)];
        
        ukf_init.ProcessNoise = Q;
        predict(ukf_init, dt(k));
    end
    
    % Position-only measurement
    z_pos = [data.x(k); data.y(k)];
    correct(ukf_init, z_pos);
    init_states(:,k) = ukf_init.State;
end

% Get better initial velocity from initialization
initial_vx = init_states(2, end);
initial_vy = init_states(4, end);
fprintf('UKF: Initialized with velocity: [%.2f, %.2f] m/s\n', initial_vx, initial_vy);

% STAGE 2: Main UKF with full measurement model
% Create main UKF with nonlinear measurement model that supports wrapping
ukf = trackingUKF(@cvStateTransitionFcn, @fullMeasurementFcnWithWrapping, ...
                  [data.x(1); initial_vx; data.y(1); initial_vy], ...
                  'HasMeasurementWrapping', true);

% Configure UKF parameters for optimal performance
ukf.Alpha = 1e-3;  % Small alpha for tight sigma point distribution
ukf.Beta = 2;      % Optimal for Gaussian distributions
ukf.Kappa = 0;     % Standard choice for this state dimension

% Initialize state covariance with better estimates
ukf.StateCovariance = diag([pos_std^2/2, vel_std^2, pos_std^2/2, vel_std^2]);

% Measurement noise - calibrated for AIS data
% Order: [x; y; SOG; COG]
R_base = diag([pos_std^2/2, pos_std^2/2, vel_std^2*1.5, deg2rad(5)^2]);

% Store initial state and covariance
x_est(:,1) = ukf.State;
P_est(:,:,1) = ukf.StateCovariance;

% Extended burn-in period with aggressive corrections
burn_in_period = 30;
burn_in_factor = 0.2;

% Main filtering loop with adaptive parameters
for k = 2:n
    % Determine current segment for adaptive parameters
    current_segment = findSegment(k, segments);
    segment_type = segment_types(current_segment);
    
    % Adjust process noise based on segment type
    local_q = q;
    if strcmp(segment_type, 'turn')
        % Higher process noise during turns
        local_q = q * 2;
    elseif strcmp(segment_type, 'accel') || strcmp(segment_type, 'decel')
        % Higher process noise during acceleration/deceleration
        local_q = q * 1.5;
    end
    
    % Create process noise based on time difference and segment type
    Q = [local_q*dt(k)^3/3, local_q*dt(k)^2/2, 0, 0;
         local_q*dt(k)^2/2, local_q*dt(k),    0, 0;
         0, 0, local_q*dt(k)^3/3, local_q*dt(k)^2/2;
         0, 0, local_q*dt(k)^2/2, local_q*dt(k)];
    
    % Set process noise before predicting
    ukf.ProcessNoise = Q;
    
    % Predict step
    predict(ukf, dt(k));
    
    % Check for missing measurements
    has_valid_position = ~isnan(data.x(k)) && ~isnan(data.y(k));
    has_valid_velocity = ~isnan(data.SOG(k)) && ~isnan(data.COG(k));
    
    % Adjust measurement noise based on segment
    R = R_base;
    if strcmp(segment_type, 'turn')
        % During turns, trust COG less, position more
        R(4,4) = R(4,4) * 2;  % Double COG uncertainty
        R(1:2,1:2) = R(1:2,1:2) * 0.5;  % Halve position uncertainty
    elseif k <= burn_in_period
        % During burn-in, gradually transition from trusting measurements more to normal
        alpha = (k / burn_in_period)^2;
        R = R * (burn_in_factor + (1-burn_in_factor) * alpha);
    end
    
    if has_valid_position
        if has_valid_velocity
            % Full measurement with proper COG wrapping
            cog_wrapped = wrapToPi(deg2rad(data.COG(k)));
            z = [data.x(k); data.y(k); data.SOG(k); cog_wrapped];
            
            % Set measurement noise
            ukf.MeasurementNoise = R;
            
            % Correct with measurement - wrapping handled by measurement function
            correct(ukf, z);
        else
            % Position-only measurement
            z = [data.x(k); data.y(k)];
            pos_R = diag([pos_std^2/2, pos_std^2/2]);
            
            % Use position-only measurement function temporarily
            ukf_temp = trackingUKF(@cvStateTransitionFcn, @positionMeasurementFcn, ukf.State);
            ukf_temp.StateCovariance = ukf.StateCovariance;
            ukf_temp.MeasurementNoise = pos_R;
            ukf_temp.Alpha = ukf.Alpha;
            ukf_temp.Beta = ukf.Beta;
            ukf_temp.Kappa = ukf.Kappa;
            
            correct(ukf_temp, z);
            
            % Transfer state back to main filter
            ukf.State = ukf_temp.State;
            ukf.StateCovariance = ukf_temp.StateCovariance;
        end
    end
    
    % Store estimates
    x_est(:,k) = ukf.State;
    P_est(:,:,k) = ukf.StateCovariance;
end

% Create output table
estimates = table();
estimates.timestamp = data.timestamp;
estimates.x_est = x_est(1,:)';
estimates.y_est = x_est(3,:)';
estimates.vx_est = x_est(2,:)';
estimates.vy_est = x_est(4,:)';
estimates.sog_est = sqrt(x_est(2,:).^2 + x_est(4,:).^2)';

% Properly handle course calculation to avoid discontinuities
estimates.cog_est = mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)';

% Calculate error statistics if ground truth is available
stats = calculateErrorStatistics(data, estimates, 'UKF');

% Plot results
plotFilterResults(data, estimates, 'Unscented Kalman Filter');

end

%% State transition function for CV model
function x = cvStateTransitionFcn(x, dt)
    % State transition function for constant velocity model
    % State: [x; vx; y; vy]
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
    x = F * x;
end

%% Full measurement function with measurement wrapping bounds
function [z, bounds] = fullMeasurementFcnWithWrapping(x)
    % Nonlinear measurement function: [x; y; SOG; COG]
    % State: [x; vx; y; vy]
    % Returns both measurement and wrapping bounds as required by HasMeasurementWrapping
    
    vx = x(2);
    vy = x(4);
    
    % Calculate SOG and COG
    sog = sqrt(vx^2 + vy^2);
    
    % Handle zero velocity case properly for course calculation
    if sog > 0.01
        cog = atan2(vy, vx);  % Returns value in [-π, π]
    else
        % When near zero velocity, course is not well defined
        cog = atan2(sign(vy)*0.01, sign(vx)*0.01);
    end
    
    z = [x(1); x(3); sog; cog];
    
    % Define wrapping bounds matrix as required by the documentation
    % Each row corresponds to a measurement: [lower_bound, upper_bound]
    bounds = [-inf, inf;    % x position (no wrapping)
              -inf, inf;    % y position (no wrapping) 
              -inf, inf;    % SOG (no wrapping)
              -pi, pi];     % COG (wraps at ±π)
end

%% Position-only measurement function for initialization
function z = positionMeasurementFcn(x)
    % Linear measurement function for position only: [x; y]
    % State: [x; vx; y; vy]
    z = [x(1); x(3)];
end

%% Helper functions (same as EKF implementation)
function [segments, segment_types] = detectMotionSegments(data)
    % Simple implementation for motion segment detection
    n = height(data);
    if n <= 50
        segments = {1:n};
        segment_types = {'constant'};
        return;
    end
    
    % Look for turns based on COG changes
    cog_diff = zeros(n-1, 1);
    for i = 2:n
        cog_diff(i-1) = angleDiff(data.COG(i), data.COG(i-1));
    end
    
    % Look for speed changes
    sog_diff = diff(data.SOG);
    
    % Detect segment boundaries
    turn_threshold = 2;     % degrees per sample
    accel_threshold = 0.2;  % m/s per sample
    
    is_turning = abs(cog_diff) > turn_threshold;
    is_accelerating = sog_diff > accel_threshold;
    is_decelerating = sog_diff < -accel_threshold;
    
    % Create segments based on motion type changes
    seg_boundaries = [1];
    seg_types = {};
    current_type = 'constant';
    
    for i = 1:n-1
        if is_turning(i)
            motion_type = 'turn';
        elseif is_accelerating(i)
            motion_type = 'accel';
        elseif is_decelerating(i)
            motion_type = 'decel';
        else
            motion_type = 'constant';
        end
        
        if ~strcmp(motion_type, current_type) || i == n-1
            seg_boundaries = [seg_boundaries, i+1];
            seg_types{end+1} = current_type;
            current_type = motion_type;
        end
    end
    
    if ~isempty(seg_types)
        seg_boundaries = [seg_boundaries, n];
        seg_types{end+1} = current_type;
    else
        seg_boundaries = [1, n];
        seg_types = {'constant'};
    end
    
    segments = {};
    for i = 1:length(seg_boundaries)-1
        segments{i} = seg_boundaries(i):seg_boundaries(i+1);
    end
    
    segment_types = seg_types;
end

function segment_idx = findSegment(point_idx, segments)
    segment_idx = 1;
    for i = 1:length(segments)
        if ismember(point_idx, segments{i})
            segment_idx = i;
            break;
        end
    end
end

function diff = angleDiff(angle1, angle2)
    % Calculate difference between two angles in degrees
    diff = mod(angle1 - angle2 + 180, 360) - 180;
end