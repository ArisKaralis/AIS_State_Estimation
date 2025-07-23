function [estimates, stats] = runUnscentedKalmanFilterCTRV(data, q)

if nargin < 2
    q = 0.1;
end

fprintf('UKF-CTRV: Process noise intensity: %.3f\n', q);

n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

x_est = zeros(5, n);
P_est = zeros(5, 5, n);

pos_std = 15;
vel_std = 1;

initial_vx = 0;
initial_vy = 0;
initial_speed = 0;
initial_yaw = 0;
initial_yaw_rate = 0;

if n > 1 && dt(2) > 0
    initial_vx = (data.x(2) - data.x(1)) / dt(2);
    initial_vy = (data.y(2) - data.y(1)) / dt(2);
    initial_speed = sqrt(initial_vx^2 + initial_vy^2);
    initial_yaw = atan2(initial_vy, initial_vx);
    
    if n > 2 && dt(3) > 0
        vx2 = (data.x(3) - data.x(2)) / dt(3);
        vy2 = (data.y(3) - data.y(2)) / dt(3);
        yaw2 = atan2(vy2, vx2);
        
        dyaw = angdiff(yaw2, initial_yaw);
        initial_yaw_rate = dyaw / dt(3);
        initial_yaw_rate = max(-0.1, min(0.1, initial_yaw_rate));
    end
end

ukf = trackingUKF(@ctrvStateTransitionFcn, @positionMeasurementFcn, ...
                  [data.x(1); data.y(1); initial_speed; initial_yaw; initial_yaw_rate]);

ukf.Alpha = 1e-3;
ukf.Beta = 2;
ukf.Kappa = 0;

ukf.StateCovariance = diag([pos_std^2/4, pos_std^2/4, vel_std^2, deg2rad(15)^2, deg2rad(2)^2]);

R_pos = diag([pos_std^2, pos_std^2]);

x_est(:,1) = ukf.State;
P_est(:,:,1) = ukf.StateCovariance;

for k = 2:n
    dt_k = dt(k);
    if dt_k <= 0
        dt_k = 1;
    end
    
    Q = q * diag([dt_k^3/3, dt_k^3/3, dt_k, deg2rad(1)^2*dt_k, deg2rad(0.5)^2*dt_k]);
    
    ukf.ProcessNoise = Q;
    
    predict(ukf, dt_k);
    
    ukf.State(4) = mod(ukf.State(4) + pi, 2*pi) - pi;
    
    has_valid_position = ~isnan(data.x(k)) && ~isnan(data.y(k));
    
    if has_valid_position
        z = [data.x(k); data.y(k)];
        ukf.MeasurementNoise = R_pos;
        correct(ukf, z);
        
        ukf.State(4) = mod(ukf.State(4) + pi, 2*pi) - pi;
    end
    
    x_est(:,k) = ukf.State;
    P_est(:,:,k) = ukf.StateCovariance;
end

vx_est = x_est(3,:) .* cos(x_est(4,:));
vy_est = x_est(3,:) .* sin(x_est(4,:));
sog_est = x_est(3,:);
cog_est = mod(x_est(4,:) * 180/pi, 360);

estimates = table(data.timestamp, x_est(1,:)', x_est(2,:)', ...
                 vx_est', vy_est', sog_est', cog_est', x_est(5,:)', ...
                 'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est', 'yaw_rate_est'});

stats = struct();
stats.filterName = 'UKF-CTRV';

if all(ismember({'x_true', 'y_true'}, data.Properties.VariableNames))
    pos_error = sqrt((estimates.x_est - data.x_true).^2 + (estimates.y_est - data.y_true).^2);
    stats.position_rmse = sqrt(mean(pos_error.^2));
    stats.position_mean = mean(pos_error);
    stats.position_max = max(pos_error);
    stats.position_std = std(pos_error);
    stats.position_errors = pos_error;
    
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
    
    if all(ismember({'sog_true', 'cog_true'}, data.Properties.VariableNames))
        stats.sog_rmse = sqrt(mean((estimates.sog_est - data.sog_true).^2));
        
        cog_error = angdiff(deg2rad(estimates.cog_est), deg2rad(data.cog_true)) * 180/pi;
        stats.cog_rmse = sqrt(mean(cog_error.^2));
    else
        stats.sog_rmse = NaN;
        stats.cog_rmse = NaN;
    end
    
    fprintf('\n===== Unscented Kalman Filter (CTRV Model) Performance Statistics =====\n');
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

try
    plotFilterResults(data, estimates, 'Unscented Kalman Filter (CTRV Model)');
catch
    fprintf('Plotting function not available or plot.mlx conflict detected.\n');
end

end

function x_pred = ctrvStateTransitionFcn(x, dt)
    px = x(1);
    py = x(2);
    v = x(3);
    yaw = x(4);
    yaw_rate = x(5);
    
    if abs(yaw_rate) < 1e-4
        px_pred = px + v * cos(yaw) * dt;
        py_pred = py + v * sin(yaw) * dt;
        yaw_pred = yaw;
    else
        px_pred = px + (v / yaw_rate) * (sin(yaw + yaw_rate * dt) - sin(yaw));
        py_pred = py + (v / yaw_rate) * (-cos(yaw + yaw_rate * dt) + cos(yaw));
        yaw_pred = yaw + yaw_rate * dt;
    end
    
    yaw_pred = mod(yaw_pred + pi, 2*pi) - pi;
    
    x_pred = [px_pred; py_pred; v; yaw_pred; yaw_rate];
end

function z = positionMeasurementFcn(x)
    z = [x(1); x(2)];
end