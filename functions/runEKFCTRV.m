function [estimates, stats] = runEKFCTRV(data, q, pos_std, vel_std)
% RUNEKFCTRV - Apply Extended Kalman Filter (CTRV model) to AIS data with NEES/NIS
if nargin < 4, vel_std = 1; end
if nargin < 3, pos_std = 15; end
if nargin < 2, q = 0.1; end

% fprintf('Starting Extended Kalman Filter (CTRV) with q=%.3f, pos_std=%.1fm, vel_std=%.1fm/s\n', q, pos_std, vel_std);

% Run filter with innovation tracking
[x_est, P_est, innovations, S_innovations] = ekfCTRV(data, q, pos_std, vel_std);

% Create estimates struct (not table) with NEES/NIS data
% CTRV State: [x, y, v, yaw, yaw_rate]
estimates = struct();
estimates.timestamp = data.timestamp;
estimates.x_est = x_est(1,:)';  % x position
estimates.y_est = x_est(2,:)';  % y position
estimates.vx_est = (x_est(3,:) .* cos(x_est(4,:)))';  % x velocity
estimates.vy_est = (x_est(3,:) .* sin(x_est(4,:)))';  % y velocity
estimates.sog_est = x_est(3,:)';  % speed over ground (v)
estimates.cog_est = mod(x_est(4,:)' * 180/pi, 360);  % course over ground (yaw)
estimates.yaw_rate_est = x_est(5,:)';  % yaw rate

% Calculate comprehensive statistics with NEES/NIS
stats = calculateFilterStats(estimates, data, 'EKF-CTRV', P_est, innovations, S_innovations);

% Add NEES and NIS data to estimates for comparison functions
if isfield(stats, 'nees_values') && ~isempty(stats.nees_values)
    estimates.nees = stats.nees_values;
else
    estimates.nees = NaN(height(data), 1);
end

if isfield(stats, 'nis_values') && ~isempty(stats.nis_values)
    estimates.nis = stats.nis_values;
else
    estimates.nis = NaN(height(data), 1);
end

% Print enhanced results
printFilterResultsEnhanced(stats);
end