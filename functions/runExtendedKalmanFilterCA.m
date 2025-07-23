function [estimates, stats] = runExtendedKalmanFilterCA(data, q)

if nargin < 2
    q = 0.001;
end

n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

x_est = zeros(6, n);
P_est = zeros(6, 6, n);

fprintf('\nEKF-CA: Process noise intensity: %.3f\n', q);

pos_std = 15;
vel_std = 1;
acc_std = 0.05;

initial_vx = 0;
initial_vy = 0;
initial_ax = 0;
initial_ay = 0;

kf = trackingKF('MotionModel', 'Custom', ...
               'StateTransitionModel', eye(6), ...
               'MeasurementModel', [1 0 0 0 0 0; 0 0 0 1 0 0], ...
               'State', [data.x(1); initial_vx; initial_ax; data.y(1); initial_vy; initial_ay], ...
               'StateCovariance', diag([pos_std^2, vel_std^2, acc_std^2, pos_std^2, vel_std^2, acc_std^2]), ...
               'MeasurementNoise', diag([pos_std^2, pos_std^2]));

x_est(:,1) = kf.State;
P_est(:,:,1) = kf.StateCovariance;

for k = 2:n
    if dt(k) <= 0 || dt(k) > 60
        dt(k) = 1.0;
    end
    
    F = [1 dt(k) dt(k)^2/2 0 0 0;
         0 1     dt(k)     0 0 0;
         0 0     1         0 0 0;
         0 0     0         1 dt(k) dt(k)^2/2;
         0 0     0         0 1     dt(k);
         0 0     0         0 0     1];
    
    Q = q * diag([dt(k)^4/4, dt(k)^3/3, dt(k)^2/2, dt(k)^4/4, dt(k)^3/3, dt(k)^2/2]);
    
    kf.StateTransitionModel = F;
    kf.ProcessNoise = Q;
    
    predict(kf);
    
    max_accel = 0.1;
    kf.State(3) = max(-max_accel, min(max_accel, kf.State(3)));
    kf.State(6) = max(-max_accel, min(max_accel, kf.State(6)));
    
    z = [data.x(k); data.y(k)];
    [x_est(:,k), P_est(:,:,k)] = correct(kf, z);
    
    x_est(3,k) = max(-max_accel, min(max_accel, x_est(3,k)));
    x_est(6,k) = max(-max_accel, min(max_accel, x_est(6,k)));
    kf.State = x_est(:,k);
end

estimates = table(data.timestamp, x_est(1,:)', x_est(4,:)', ...
                 x_est(2,:)', x_est(5,:)', x_est(3,:)', x_est(6,:)', ...
                 sqrt(x_est(2,:).^2 + x_est(5,:).^2)', ...
                 mod(atan2(x_est(5,:), x_est(2,:)) * 180/pi, 360)', ...
                 'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'ax_est', 'ay_est', 'sog_est', 'cog_est'});

stats = struct();
try
    if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
        pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
        stats.position_rmse = sqrt(mean(pos_error.^2));
        stats.position_mean = mean(pos_error);
        stats.position_max = max(pos_error);
        stats.position_std = std(pos_error);
        stats.position_errors = pos_error;
    else
        stats.position_rmse = NaN;
        stats.position_mean = NaN;
        stats.position_max = NaN;
        stats.position_std = NaN;
    end
catch
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
    stats.position_std = NaN;
end

% Check if ground truth is available
if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    % Position errors
    pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    stats.position_rmse = sqrt(mean(pos_error.^2));
    stats.position_mean = mean(pos_error);
    stats.position_max = max(pos_error);
    stats.position_std = std(pos_error);
    stats.position_errors = pos_error;
    
    % Print result
    fprintf('\n===== Extended Kalman Filter (CA Model) Performance Statistics =====\n');
    fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
    fprintf('Position Max Error: %.2f m\n', stats.position_max);
else
    fprintf('No ground truth available for error calculation.\n');
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
end


plotFilterResults(data, estimates, 'Extended Kalman Filter (CA Model)');


end

