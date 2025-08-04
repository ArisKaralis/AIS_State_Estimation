function [estimates, stats] = runEKFCTRV(data, q, pos_std, vel_std)

if nargin < 2
    q = 0.1;
end

fprintf('\nEKF-CTRV: Process noise intensity: %.3f\n', q);

 

% Run core filter
[x_est, P_est] = ekfCTRV(data, q, pos_std, vel_std);

% Create estimates table
estimates = createEstimatesTable(data.timestamp, x_est, 'ctrv');

% Calculate statistics
stats = calculateFilterStats(estimates, data, 'EKF-CTRV');

% Print results
printFilterResults(stats);

% Plot results
% plotFilterResults(data, estimates, 'Extended Kalman Filter (CTRV Model)');


end