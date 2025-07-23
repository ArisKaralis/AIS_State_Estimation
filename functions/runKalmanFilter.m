function [estimates, stats] = runKalmanFilter(data, q)
% RUNKALMANFILTER - Apply CV Kalman Filter to AIS data
% Uses trackingKF from Sensor Fusion and Tracking Toolbox

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    % Convert duration to numeric seconds - more robust approach
    time_diff = data.timestamp(i) - data.timestamp(i-1);
    if isduration(time_diff)
        dt(i) = double(seconds(time_diff)); % Convert duration to numeric seconds
    else
        dt(i) = seconds(time_diff); % This might return a duration object
        if isduration(dt(i))
            dt(i) = double(dt(i)); % Convert duration to numeric seconds
        end
    end
end

% Initialize state and covariance arrays
x_est = zeros(4, n);
P_est = zeros(4, 4, n);

% Process noise intensity (tuning parameter)
% fprintf('\nProcess noise intensity: %.3f\n', q);

% Get initial velocity from first two measurements if possible
initial_vx = 0;
initial_vy = 0;
if n > 1 && dt(2) > 0
    initial_vx = (data.x(2) - data.x(1)) / dt(2);
    initial_vy = (data.y(2) - data.y(1)) / dt(2);
end

% Use the measured position standard deviation from your simulation (about 21m RMSE)
% RMSE = 21 means standard deviation is approximately 21/sqrt(2) ≈ 15m for each axis
pos_std = 15;  
vel_std = 1;   % From SOG RMSE of ~1 m/s

% Create Kalman filter with custom motion model
% State: [x; vx; y; vy]
% Measurement: [x; y]
kf = trackingKF('MotionModel', 'Custom', ...
               'StateTransitionModel', eye(4), ... % Will be updated in the loop
               'MeasurementModel', [1 0 0 0; 0 0 1 0], ...
               'State', [data.x(1); initial_vx; data.y(1); initial_vy], ...
               'StateCovariance', diag([pos_std^2, vel_std^2, pos_std^2, vel_std^2]), ...
               'MeasurementNoise', diag([pos_std^2, pos_std^2]));  % Using measured noise stats

% Process first measurement
z = [data.x(1); data.y(1)];
[x_est(:,1), P_est(:,:,1)] = correct(kf, z);

% Main filtering loop
for k = 2:n
    % Ensure dt(k) is numeric before using in calculations
    dt_k = dt(k);
    if isduration(dt_k)
        dt_k = double(dt_k);
    end
    
    % Time update (prediction)
    % Adjust state transition and process noise based on time difference
    F = [1 dt_k 0 0;
         0 1    0 0;
         0 0    1 dt_k;
         0 0    0 1];
    
    % Use discretized continuous white noise acceleration model
    Q = q*[dt_k^3/3, dt_k^2/2, 0, 0;
         dt_k^2/2, dt_k,    0, 0;
         0, 0, dt_k^3/3, dt_k^2/2;
         0, 0, dt_k^2/2, dt_k];
    
    % Update the filter parameters before predicting
    kf.StateTransitionModel = F;
    kf.ProcessNoise = Q;
    
    predict(kf);
    
    % Measurement update
    z = [data.x(k); data.y(k)];
    [x_est(:,k), P_est(:,:,k)] = correct(kf, z);
end

% Create output table
estimates = table();
estimates.timestamp = data.timestamp;
estimates.x_est = x_est(1,:)';
estimates.y_est = x_est(3,:)';
estimates.vx_est = x_est(2,:)';
estimates.vy_est = x_est(4,:)';
estimates.sog_est = sqrt(x_est(2,:).^2 + x_est(4,:).^2)';
estimates.cog_est = mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)';

% Calculate error statistics if ground truth is available
% Avoid calling calculateErrorStatistics to prevent circular dependency
stats = struct();
stats.filterName = 'KF';

% Check if ground truth is available
if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    % Position errors
    pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    stats.position_rmse = sqrt(mean(pos_error.^2));
    stats.position_mean = mean(pos_error);
    stats.position_max = max(pos_error);
    stats.position_std = std(pos_error);
    stats.position_errors = pos_error;
    
    % Print results
    fprintf('\n===== Kalman Filter (CV Model) Performance Statistics =====\n');
    fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
    fprintf('Position Max Error: %.2f m\n', stats.position_max);
else
    fprintf('No ground truth available for error calculation.\n');
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
end

% Plot results
plotFilterResults(data, estimates, 'Kalman Filter (CV Model)');

end