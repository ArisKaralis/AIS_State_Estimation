function [estimates, stats] = runExtendedKalmanFilterCV(data, q)
% RUNEXTENDEDKALMANFILTERCV - Apply EKF with CV model to AIS data
% Uses extendedKalmanFilter from Sensor Fusion and Tracking Toolbox
% Input:
%   data - AIS data table
%   q - Process noise intensity parameter (optional, default=0.05)

% Set default process noise if not provided
if nargin < 2
    q = 0.05;
end

fprintf('Using process noise intensity q = %.3f\n', q);

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
% This helps with adaptive filtering parameters
[segments, segment_types] = detectMotionSegments(data);
fprintf('Detected %d motion segments in the data\n', length(segments));

% Two-stage filtering approach: first filter without COG to get better initial velocity
% This helps avoid the poor initial convergence

% STAGE 1: Run a simplified KF on position-only data for initialization
kf_init = trackingKF('MotionModel', 'Custom', ...
                   'StateTransitionModel', eye(4), ...
                   'MeasurementModel', [1 0 0 0; 0 0 1 0], ...
                   'State', [data.x(1); 0; data.y(1); 0], ...
                   'StateCovariance', diag([pos_std^2, 100, pos_std^2, 100]), ...
                   'MeasurementNoise', diag([pos_std^2, pos_std^2]));

% Run the initialization filter for the first segment only
init_length = min(30, n); % Just first 30 points or whole dataset
init_states = zeros(4, init_length);

for k = 1:init_length
    % Update state transition and process noise for current time step
    if k > 1
        F = [1 dt(k) 0 0;
             0 1    0 0;
             0 0    1 dt(k);
             0 0    0 1];
        
        Q = [q*dt(k)^3/3, q*dt(k)^2/2, 0, 0;
             q*dt(k)^2/2, q*dt(k),    0, 0;
             0, 0, q*dt(k)^3/3, q*dt(k)^2/2;
             0, 0, q*dt(k)^2/2, q*dt(k)];
        
        kf_init.StateTransitionModel = F;
        kf_init.ProcessNoise = Q;
        predict(kf_init);
    end
    
    % Use position-only measurements
    z = [data.x(k); data.y(k)];
    [init_states(:,k), ~] = correct(kf_init, z);
end

% Get better initial velocity from the initialization filter
initial_vx = init_states(2, end);
initial_vy = init_states(4, end);
fprintf('Initialized with velocity: [%.2f, %.2f] m/s\n', initial_vx, initial_vy);

% STAGE 2: Main EKF with all measurements
% Create EKF using CV model with nonlinear measurement model
% State: [x; vx; y; vy]
% Measurement: [x; y; SOG; COG]
ekf = extendedKalmanFilter(@cvStateTransitionFcn, @measurementFcn);

% Setting StateTransitionJacobianFcn and MeasurementJacobianFcn to improve accuracy
ekf.StateTransitionJacobianFcn = @stateTransitionJacobianFcn;
ekf.MeasurementJacobianFcn = @measurementJacobianFcn;

% Initialize filter state using the initialization filter results
ekf.State = [data.x(1); initial_vx; data.y(1); initial_vy];

% Use tighter initial covariance based on pre-filtering
ekf.StateCovariance = diag([pos_std^2/2, vel_std^2*2, pos_std^2/2, vel_std^2*2]);

% Measurement noise - calibrated for AIS data
% More trust in position, less in velocity
R_base = diag([pos_std^2/2, pos_std^2/2, vel_std^2*1.5, deg2rad(5)^2]);

% Store initial state and covariance
x_est(:,1) = ekf.State;
P_est(:,:,1) = ekf.StateCovariance;

% Extended burn-in period with aggressive corrections
burn_in_period = 30;
burn_in_factor = 0.2; % Lower noise = trust measurements more during burn-in

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
    ekf.ProcessNoise = Q;
    
    % Predict step - passing dt(k) to the state transition function
    predict(ekf, dt(k));
    
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
        alpha = (k / burn_in_period)^2; % Nonlinear transition
        R = R * (burn_in_factor + (1-burn_in_factor) * alpha);
    end
    
    if has_valid_position
        % Create appropriate measurement vector
        if has_valid_velocity
            % Process COG to handle circular measurements correctly
            cog_rad = unwrapCOG(data.COG(k), ekf.State);
            
            % Full measurement: [x; y; SOG; COG]
            z = [data.x(k); data.y(k); data.SOG(k); cog_rad];
            
            % Full correction
            ekf.MeasurementNoise = R;
            correct(ekf, z);
        else
            % Position-only measurement: [x; y]
            z = [data.x(k); data.y(k)];
            
            % Position-only noise
            pos_R = diag([pos_std^2/2, pos_std^2/2]);
            
            % Use only position components for correction
            correct(ekf, z, [1, 2], pos_R);
        end
    end
    
    % Store estimates
    x_est(:,k) = ekf.State;
    P_est(:,:,k) = ekf.StateCovariance;
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
stats = calculateErrorStatistics(data, estimates, 'EKF-CV');

% Plot results
plotFilterResults(data, estimates, 'Extended Kalman Filter (CV Model)');

end

% State transition function for CV model that takes dt as parameter
function x = cvStateTransitionFcn(x, dt)
    % State transition function for constant velocity model
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
    x = F * x;
end

% State transition Jacobian function that returns the Jacobian matrix
function F = stateTransitionJacobianFcn(x, dt)
    % Jacobian of state transition function with respect to state
    % For CV model, this is just the state transition matrix
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
end

% Measurement function that converts state to measurement
function z = measurementFcn(x)
    % Nonlinear measurement function: [x; y; SOG; COG]
    vx = x(2);
    vy = x(4);
    
    % Calculate SOG and COG
    sog = sqrt(vx^2 + vy^2);
    
    % Handle zero velocity case properly for course calculation
    if sog > 0.01
        cog = atan2(vy, vx);
    else
        % When near zero velocity, course is not well defined
        % Use a small default velocity to avoid instability
        cog = atan2(sign(vy)*0.01, sign(vx)*0.01);
    end
    
    z = [x(1); x(3); sog; cog];
end

% Measurement Jacobian function that returns the Jacobian matrix
function H = measurementJacobianFcn(x)
    % Jacobian of measurement function with respect to state
    vx = x(2);
    vy = x(4);
    speed = sqrt(vx^2 + vy^2);
    
    % Position derivatives are simple
    H = zeros(4, 4);
    H(1, 1) = 1;  % dx/dx
    H(2, 3) = 1;  % dy/dy
    
    % SOG derivatives
    if speed > 0.01
        H(3, 2) = vx / speed;  % d(SOG)/d(vx)
        H(3, 4) = vy / speed;  % d(SOG)/d(vy)
        
        % COG derivatives - handle carefully for numerical stability
        denominator = vx^2 + vy^2;
        H(4, 2) = -vy / denominator;  % d(COG)/d(vx)
        H(4, 4) = vx / denominator;   % d(COG)/d(vy)
    else
        % When speed is near zero, linearize around small non-zero values
        % to avoid numeric instabilities
        H(3, 2) = 1;  % Small velocity in x direction
        H(3, 4) = 0;
        H(4, 2) = 0;
        H(4, 4) = 1;  % Course changes primarily with y velocity at low speed
    end
end

% Helper function to detect motion segments in the data
function [segments, segment_types] = detectMotionSegments(data)
    % Simple implementation - you can make this more sophisticated
    % based on your actual data characteristics
    
    n = height(data);
    if n <= 50
        % If data is short, just use one segment
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
    turn_threshold = 2;   % degrees per sample
    accel_threshold = 0.2;  % m/s per sample
    
    is_turning = abs(cog_diff) > turn_threshold;
    is_accelerating = sog_diff > accel_threshold;
    is_decelerating = sog_diff < -accel_threshold;
    
    % Segment boundaries (add 1 for indexing)
    seg_boundaries = [1];
    seg_types = {};
    
    % Current segment type
    current_type = 'constant';
    
    % Scan through the data to identify segments
    for i = 1:n-1
        % Determine motion type at this point
        if is_turning(i)
            motion_type = 'turn';
        elseif is_accelerating(i)
            motion_type = 'accel';
        elseif is_decelerating(i)
            motion_type = 'decel';
        else
            motion_type = 'constant';
        end
        
        % If type changes, start a new segment
        if ~strcmp(motion_type, current_type) || i == n-1
            seg_boundaries = [seg_boundaries, i+1];
            seg_types{end+1} = current_type;
            current_type = motion_type;
        end
    end
    
    % Add the last segment
    if ~isempty(seg_types)
        seg_boundaries = [seg_boundaries, n];
        seg_types{end+1} = current_type;
    else
        % If no segments were detected, use the whole dataset
        seg_boundaries = [1, n];
        seg_types = {'constant'};
    end
    
    % Create segment ranges
    segments = {};
    for i = 1:length(seg_boundaries)-1
        segments{i} = seg_boundaries(i):seg_boundaries(i+1);
    end
    
    segment_types = seg_types;
end

% Helper function to find which segment a point belongs to
function segment_idx = findSegment(point_idx, segments)
    segment_idx = 1;  % Default to first segment
    for i = 1:length(segments)
        if ismember(point_idx, segments{i})
            segment_idx = i;
            break;
        end
    end
end

% Helper function to properly unwrap COG measurements for the filter
function cog_rad = unwrapCOG(cog_deg, state)
    % Convert COG from degrees to radians
    cog_rad = deg2rad(cog_deg);
    
    % Get the predicted COG from the state
    vx = state(2);
    vy = state(4);
    if norm([vx, vy]) > 0.01
        pred_cog = atan2(vy, vx);
    else
        pred_cog = 0;
    end
    
    % Unwrap the COG measurement to be close to the predicted COG
    % This handles the circular nature of angles
    while cog_rad - pred_cog > pi
        cog_rad = cog_rad - 2*pi;
    end
    while cog_rad - pred_cog < -pi
        cog_rad = cog_rad + 2*pi;
    end
end

% Helper function to calculate angular difference correctly
function diff = angleDiff(angle1, angle2)
    % Calculate difference between two angles in degrees, 
    % accounting for circular wrapping
    diff = mod(angle1 - angle2 + 180, 360) - 180;
end