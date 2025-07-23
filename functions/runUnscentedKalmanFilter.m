function [estimates, stats] = runUnscentedKalmanFilter(data, q)

if nargin < 2
    q = 0.1;
end

fprintf('UKF: Using process noise intensity q = %.3f\n', q);

n = height(data);
dt = zeros(n, 1);
for i = 2:n
    dt(i) = seconds(data.timestamp(i) - data.timestamp(i-1));
end

x_est = zeros(4, n);
P_est = zeros(4, 4, n);

pos_std = 30;
vel_std = 2;

initial_vx = 0;
initial_vy = 0;

ukf = trackingUKF(@cvStateTransitionFcn, @fullMeasurementFcn, ...
                  [data.x(1); initial_vx; data.y(1); initial_vy]);

ukf.Alpha = 1;
ukf.Beta = 5;
ukf.Kappa = 0.10;
ukf.StateCovariance = diag([pos_std^2, vel_std^2, pos_std^2, vel_std^2]);

R = diag([pos_std^2, pos_std^2, vel_std^2, deg2rad(5)^2]);

x_est(:,1) = ukf.State;
P_est(:,:,1) = ukf.StateCovariance;

for k = 2:n
    Q = [q*dt(k)^3/3, q*dt(k)^2/2, 0, 0;
         q*dt(k)^2/2, q*dt(k),    0, 0;
         0, 0, q*dt(k)^3/3, q*dt(k)^2/2;
         0, 0, q*dt(k)^2/2, q*dt(k)];
    
    ukf.ProcessNoise = Q;
    
    predict(ukf, dt(k));
    
    has_valid_position = ~isnan(data.x(k)) && ~isnan(data.y(k));
    has_valid_velocity = ~isnan(data.SOG(k)) && ~isnan(data.COG(k));
    
    if has_valid_position
        if has_valid_velocity
            cog_wrapped = wrapToPi(deg2rad(data.COG(k)));
            z = [data.x(k); data.y(k); data.SOG(k); cog_wrapped];
            ukf.MeasurementNoise = R;
            correct(ukf, z);
        else
            ukf_temp = trackingUKF(@cvStateTransitionFcn, @positionMeasurementFcn, ukf.State);
            ukf_temp.StateCovariance = ukf.StateCovariance;
            ukf_temp.Alpha = ukf.Alpha;
            ukf_temp.Beta = ukf.Beta;
            ukf_temp.Kappa = ukf.Kappa;
            
            z = [data.x(k); data.y(k)];
            pos_R = diag([pos_std^2, pos_std^2]);
            ukf_temp.MeasurementNoise = pos_R;
            correct(ukf_temp, z);
            
            ukf.State = ukf_temp.State;
            ukf.StateCovariance = ukf_temp.StateCovariance;
        end
    end
    
    x_est(:,k) = ukf.State;
    P_est(:,:,k) = ukf.StateCovariance;
end

estimates = table(data.timestamp, x_est(1,:)', x_est(3,:)', ...
                 x_est(2,:)', x_est(4,:)', ...
                 sqrt(x_est(2,:).^2 + x_est(4,:).^2)', ...
                 mod(atan2(x_est(4,:), x_est(2,:)) * 180/pi, 360)', ...
                 'VariableNames', {'timestamp', 'x_est', 'y_est', 'vx_est', 'vy_est', 'sog_est', 'cog_est'});

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
    fprintf('\n===== Unscented Kalman Filter (CV Model) Performance Statistics =====\n');
    fprintf('Position RMSE: %.2f m\n', stats.position_rmse);
    fprintf('Position Mean Error: %.2f m\n', stats.position_mean);
    fprintf('Position Max Error: %.2f m\n', stats.position_max);
else
    fprintf('No ground truth available for error calculation.\n');
    stats.position_rmse = NaN;
    stats.position_mean = NaN;
    stats.position_max = NaN;
end


plotFilterResults(data, estimates, 'Unscented Kalman Filter (CV Model)');


end

function x = cvStateTransitionFcn(x, dt)
    F = [1 dt 0 0;
         0 1  0 0;
         0 0  1 dt;
         0 0  0 1];
    x = F * x;
end

function z = fullMeasurementFcn(x)
    vx = x(2);
    vy = x(4);
    sog = sqrt(vx^2 + vy^2);
    cog = atan2(vy, vx);
    z = [x(1); x(3); sog; cog];
end

function z = positionMeasurementFcn(x)
    z = [x(1); x(3)];
end