function [estimates, stats] = runUKFCTRV(data, q, pos_std, vel_std)


if nargin < 2
    q = 0.1;
end

fprintf('UKF-CTRV: Process noise intensity: %.3f\n', q);
 

% Run core filter
[x_est, P_est] = ukfCTRV(data, q, pos_std, vel_std);

% Create estimates table
estimates = createEstimatesTable(data.timestamp, x_est, 'ctrv');

% Calculate statistics
stats = calculateFilterStats(estimates, data, 'UKF-CTRV');

% Print results
printFilterResults(stats);

% Plot results
% plotFilterResults(data, estimates, 'Unscented Kalman Filter (CTRV Model)');


end