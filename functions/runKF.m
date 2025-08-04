function [estimates, stats] = runKF(data, q, pos_std, vel_std)

if nargin < 2
    q = 0.1;
end

fprintf('KF: Process noise intensity: %.3f\n', q);

% Run core filter
[x_est, P_est] = kfCV(data, q, pos_std, vel_std);

% Create estimates table
estimates = createEstimatesTable(data.timestamp, x_est, 'cv');

% Calculate statistics
stats = calculateFilterStats(estimates, data, 'KF');

% Print results
printFilterResults(stats);

% Plot results
% plotFilterResults(data, estimates, 'Kalman Filter (CV Model)');


end