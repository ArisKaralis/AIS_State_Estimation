function [estimates, stats] = runKalmanFilter(data, q)
% RUNKALMANFILTER - Apply CV Kalman Filter to AIS data
% Uses trackingKF from Sensor Fusion and Tracking Toolbox

% Extract measurements
n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

% Initialize state and covariance arrays
x_est = zeros(4, n);
P_est = zeros(4, 4, n);

% Process noise intensity (tuning parameter)
fprintf('\nProcess noise intensity: %.3f\n', q);

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
    % Time update (prediction)
    % Adjust state transition and process noise based on time difference
    F = [1 dt(k) 0 0;
         0 1    0 0;
         0 0    1 dt(k);
         0 0    0 1];
    
    % Use discretized continuous white noise acceleration model
    Q = [q*dt(k)^3/3, q*dt(k)^2/2, 0, 0;
         q*dt(k)^2/2, q*dt(k),    0, 0;
         0, 0, q*dt(k)^3/3, q*dt(k)^2/2;
         0, 0, q*dt(k)^2/2, q*dt(k)];
    
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
stats = calculateErrorStatistics(data, estimates, 'KF');

% Plot results
plotFilterResults(data, estimates, 'Kalman Filter (CV Model)');

end