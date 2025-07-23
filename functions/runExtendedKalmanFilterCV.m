function [estimates, stats] = runExtendedKalmanFilterCV(data, q)
% RUNEXTENDEDKALMANFILTERCV - Apply EKF-CV to AIS data
% Improved version following successful UKF approach

if nargin < 2
    q = 0.1;
end

n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

x_est = zeros(4, n);
P_est = zeros(4, 4, n);

fprintf('\nEKF-CV: Process noise intensity: %.3f\n', q);

% Use same noise parameters as successful UKF
pos_std = 15;  % Reduced from 30
vel_std = 1;   % Reduced from 2

% Better initial velocity estimation
initial_vx = 0;
initial_vy = 0;
if n > 1 && dt(2) > 0
    initial_vx = (data.x(2) - data.x(1)) / dt(2);
    initial_vy = (data.y(2) - data.y(1)) / dt(2);
end

% Create EKF with simplified position-only measurements (like UKF)
ekf = extendedKalmanFilter(@cvStateTransitionFcn, @measurementFcn);
ekf.StateTransitionJacobianFcn = @stateTransitionJacobianFcn;
ekf.MeasurementJacobianFcn = @measurementJacobianFcn;

% Initialize with better state and covariance
ekf.State = [data.x(1); initial_vx; data.y(1); initial_vy];
ekf.StateCovariance = diag([pos_std^2/4, vel_std^2, pos_std^2/4, vel_std^2]); % More confident like UKF

% Position-only measurement noise (like UKF)
R_pos = diag([pos_std^2, pos_std^2]);

% Store initial estimates
x_est(:,1) = ekf.State;
P_est(:,:,1) = ekf.StateCovariance;

% Main filtering loop
for k = 2:n
    % Ensure positive time step
    dt_k = dt(k);
    if dt_k <= 0
        dt_k = 1; % Fallback for bad timestamps
    end
    
    % Process noise matrix (same structure as UKF)
    Q = q * [dt_k^3/3, dt_k^2/2, 0, 0;
             dt_k^2/2, dt_k,     0, 0;
             0,        0,        dt_k^3/3, dt_k^2/2;
             0,        0,        dt_k^2/2, dt_k];
    
    ekf.ProcessNoise = Q;
    
    % Prediction step
    predict(ekf, dt_k);
    
    % Simple position-only measurement update (like UKF)
    has_valid_position = ~isnan(data.x(k)) && ~isnan(data.y(k));
    
    if has_valid_position
        z = [data.x(k); data.y(k)];
        ekf.MeasurementNoise = R_pos;
        correct(ekf, z);
    end
    
    % Store estimates
    x_est(:,k) = ekf.State;
    P_est(:,:,k) = ekf.StateCovariance;
end

% Create output table
estimates = table(data.timestamp, x_est(1,:)', x_est(3,:)', ...
                 x_est(2,:)', x_est(4,:)', ...
                 sqrt(x_est(2,:).^2 + x_est(4,:).^2)', ...
                 mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)', ...
                 'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'});

% Calculate error statistics
stats = struct();
stats.filterName = 'EKF-CV';

if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    % Position errors
    pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    stats.position_rmse = sqrt(mean(pos_error.^2));
    stats.position_mean = mean(pos_error);
    stats.position_max = max(pos_error);
    stats.position_std = std(pos_error);
    stats.position_errors = pos_error;
    
    % Additional statistics if available
    if all(ismember({'vx_true', 'vy_true'}, data.Properties.VariableNames))
        vel_error = sqrt((estimates.vx_est - data.vx_true).^2 + (estimates.vy_est - data.vy_true).^2);
        stats.velocity_rmse = sqrt(mean(vel_error.^2));
        stats.velocity_mean = mean(vel_error);
        stats.velocity_max = max(vel_error);
    end
    
    if all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames))
        stats.sog_rmse = sqrt(mean((estimates.sog_est - data.sog_true).^2));
        stats.cog_rmse = sqrt(mean((angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true)) * 180/pi).^2));
    end
    
    % Print results
    fprintf('\n===== Extended Kalman Filter (CV Model) Performance Statistics =====\n');
    fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
    fprintf('Position Max Error: %.2f m\n', stats.position_max);
    
    if isfield(stats, 'velocity_rmse')
        fprintf('Velocity RMSE: %.2f m/s\n', stats.velocity_rmse);
    end
    if isfield(stats, 'sog_rmse')
        fprintf('SOG RMSE: %.2f m/s\n', stats.sog_rmse);
        fprintf('COG RMSE: %.2f degrees\n', stats.cog_rmse);
    end
else
    fprintf('No ground truth available for error calculation.\n');
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
end

% Plot results if plotting function is available
try
    plotFilterResults(data, estimates, 'Extended Kalman Filter (CV Model)');
catch
    fprintf('Plotting function not available or plot.mlx conflict detected.\n');
end

end

% State transition function for CV model
function x = cvStateTransitionFcn(x, dt)
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
    x = F * x;
end

% State transition Jacobian
function F = stateTransitionJacobianFcn(x, dt)
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
end

% Simple position-only measurement function (like UKF)
function z = measurementFcn(x)
    z = [x(1); x(3)]; % Only position measurements
end

% Simple position-only measurement Jacobian
function H = measurementJacobianFcn(x)
    H = [1 0 0 0;
         0 0 1 0]; % Only position measurements
end