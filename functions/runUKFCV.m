function [estimates, stats] = runUKFCV(data, q, pos_std, vel_std)

if nargin < 2
    q = 0.1;
end

fprintf('UKF: Using process noise intensity q = %.3f\n', q);


% Run core filter
[x_est, P_est] = ukfCV(data, q, pos_std, vel_std);

% Create estimates table
estimates = createEstimatesTable(data.timestamp, x_est, 'cv');

% Calculate statistics
stats = calculateFilterStats(estimates, data, 'UKF-CV');

% Print results
printFilterResults(stats);

% Plot results
% plotFilterResults(data, estimates, 'Unscented Kalman Filter (CV Model)');

end