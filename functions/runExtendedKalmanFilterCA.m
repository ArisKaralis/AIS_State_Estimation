function [estimates, stats] = runExtendedKalmanFilterCA(data, q)
% RUNEXTENDEDKALMANFILTERCA - Apply EKF-CA to AIS data
% Improved version following successful UKF/EKF-CV approach

if nargin < 2
    q = 0.1;
end

n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

x_est = zeros(6, n);
P_est = zeros(6, 6, n);

fprintf('\nEKF-CA: Process noise intensity: %.3f\n', q);

% Use same noise parameters as successful filters
pos_std = 15;    % Same as UKF/EKF-CV
vel_std = 1;     % Same as UKF/EKF-CV  
acc_std = 0.1;   % Increased from 0.05 - marine vessels can accelerate more

% Better initial conditions - estimate velocity and acceleration from position data
initial_vx = 0;
initial_vy = 0;
initial_ax = 0;
initial_ay = 0;

% Estimate initial velocity from first few points
if n > 1 && dt(2) > 0
    initial_vx = (data.x(2) - data.x(1)) / dt(2);
    initial_vy = (data.y(2) - data.y(1)) / dt(2);
end

% Estimate initial acceleration from velocity changes
if n > 2 && dt(3) > 0
    vx2 = (data.x(3) - data.x(2)) / dt(3);
    vy2 = (data.y(3) - data.y(2)) / dt(3);
    initial_ax = (vx2 - initial_vx) / dt(3);
    initial_ay = (vy2 - initial_vy) / dt(3);
    % Limit initial acceleration to reasonable values
    initial_ax = max(-0.5, min(0.5, initial_ax));
    initial_ay = max(-0.5, min(0.5, initial_ay));
end

% Create Kalman filter with CA model
% State: [x, vx, ax, y, vy, ay]
kf = trackingKF('MotionModel', 'Custom', ...
               'StateTransitionModel', eye(6), ...
               'MeasurementModel', [1 0 0 0 0 0; 0 0 0 1 0 0], ...
               'State', [data.x(1); initial_vx; initial_ax; data.y(1); initial_vy; initial_ay], ...
               'StateCovariance', diag([pos_std^2/4, vel_std^2, acc_std^2, pos_std^2/4, vel_std^2, acc_std^2]), ...
               'MeasurementNoise', diag([pos_std^2, pos_std^2]));

% Store initial estimates
x_est(:,1) = kf.State;
P_est(:,:,1) = kf.StateCovariance;

% Main filtering loop
for k = 2:n
    % Ensure positive time step
    dt_k = dt(k);
    if dt_k <= 0 || dt_k > 60
        dt_k = 1.0;
    end
    
    % State transition matrix for CA model
    F = [1 dt_k dt_k^2/2 0 0    0;
         0 1    dt_k     0 0    0;
         0 0    1        0 0    0;
         0 0    0        1 dt_k dt_k^2/2;
         0 0    0        0 1    dt_k;
         0 0    0        0 0    1];
    
    % Improved process noise matrix - proper kinematic structure
    % Based on continuous white noise acceleration model
    Q11 = q * [dt_k^5/20, dt_k^4/8,  dt_k^3/6;
               dt_k^4/8,  dt_k^3/3,  dt_k^2/2;
               dt_k^3/6,  dt_k^2/2,  dt_k];
    
    Q = blkdiag(Q11, Q11); % Same structure for x and y dimensions
    
    kf.StateTransitionModel = F;
    kf.ProcessNoise = Q;
    
    predict(kf);
    
    % Reasonable acceleration limits (marine vessels can accelerate more)
    max_accel = 0.5; % Increased from 0.1
    kf.State(3) = max(-max_accel, min(max_accel, kf.State(3)));
    kf.State(6) = max(-max_accel, min(max_accel, kf.State(6)));
    
    % Measurement update - position only
    has_valid_position = ~isnan(data.x(k)) && ~isnan(data.y(k));
    
    if has_valid_position
        z = [data.x(k); data.y(k)];
        [x_est(:,k), P_est(:,:,k)] = correct(kf, z);
        
        % Apply acceleration limits after update as well
        x_est(3,k) = max(-max_accel, min(max_accel, x_est(3,k)));
        x_est(6,k) = max(-max_accel, min(max_accel, x_est(6,k)));
        kf.State = x_est(:,k);
    else
        % No measurement - just store prediction
        x_est(:,k) = kf.State;
        P_est(:,:,k) = kf.StateCovariance;
    end
end

% Create output table
estimates = table(data.timestamp, x_est(1,:)', x_est(4,:)', ...
                 x_est(2,:)', x_est(5,:)', x_est(3,:)', x_est(6,:)', ...
                 sqrt(x_est(2,:).^2 + x_est(5,:).^2)', ...
                 mod(atan2(x_est(5,:), x_est(2,:)) * 180/pi, 360)', ...
                 'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'ax_est', 'ay_est', 'sog_est', 'cog_est'});

% Calculate comprehensive error statistics
stats = struct();
stats.filterName = 'EKF-CA';

if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    % Position errors
    pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    stats.position_rmse = sqrt(mean(pos_error.^2));
    stats.position_mean = mean(pos_error);
    stats.position_max = max(pos_error);
    stats.position_std = std(pos_error);
    stats.position_errors = pos_error;
    
    % Velocity errors if available
    if all(ismember({'vx_true', 'vy_true'}, data.Properties.VariableNames))
        vel_error = sqrt((estimates.vx_est - data.vx_true).^2 + (estimates.vy_est - data.vy_true).^2);
        stats.velocity_rmse = sqrt(mean(vel_error.^2));
        stats.velocity_mean = mean(vel_error);
        stats.velocity_max = max(vel_error);
    else
        stats.velocity_rmse = NaN;
        stats.velocity_mean = NaN;
        stats.velocity_max = NaN;
    end
    
    % SOG and COG errors if available
    if all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames))
        stats.sog_rmse = sqrt(mean((estimates.sog_est - data.sog_true).^2));
        stats.cog_rmse = sqrt(mean((angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
    else
        stats.sog_rmse = NaN;
        stats.cog_rmse = NaN;
    end
    
    % Print comprehensive results
    fprintf('\n===== Extended Kalman Filter (CA Model) Performance Statistics =====\n');
    fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
    fprintf('Position Max Error: %.2f m\n', stats.position_max);
    
    if ~isnan(stats.velocity_rmse)
        fprintf('Velocity RMSE: %.2f m/s\n', stats.velocity_rmse);
    end
    if ~isnan(stats.sog_rmse)
        fprintf('SOG RMSE: %.2f m/s\n', stats.sog_rmse);
        fprintf('COG RMSE: %.2f degrees\n', stats.cog_rmse);
    end
else
    fprintf('No ground truth available for error calculation.\n');
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
    stats.velocity_rmse = NaN;
    stats.velocity_mean = NaN;
    stats.velocity_max = NaN;
    stats.sog_rmse = NaN;
    stats.cog_rmse = NaN;
end

% Plot results if plotting function is available
try
    plotFilterResults(data, estimates, 'Extended Kalman Filter (CA Model)');
catch
    fprintf('Plotting function not available or plot.mlx conflict detected.\n');
end

end